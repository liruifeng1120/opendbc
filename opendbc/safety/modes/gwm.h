#pragma once

#include "opendbc/safety/safety_declarations.h"

// ===================== GWM (angle control) safety =====================
//
// Lateral control:  IDCU -> EPS on ADAS_16 (0x19c), signal ADAS_StrAngleReq
//                    (start_bit 8, 16 bits, factor 0.1 deg => 1 deg = 10 CAN units, offset -780)
// Longitudinal:      IDCU -> VCU/IBCS on ADAS_14 (0x186), signal ADAS_LongCtrlTargetAccel
//                    (start_bit 0, 8 bits, factor 0.05, offset -5 => CAN = (a+5)*20)
//
// ADAS_3 state:      IDCU -> bus0 (0x223), signals:
//                    ADAS_ACC_WorkSt    (DBC start_bit 30, size 4, lsb 27)
//                    ADAS_TJA_LCC_Workst (DBC start_bit 22, size 3, lsb 20)
//
// Checksum: GWM uses a simple checksum on all 8-byte messages:
//           checksum = (sum of bytes 0-6) ^ 0xFF  => stored in byte 7
//           Validated on 50+ message types across the V09 EV CAN bus.
//
// All signal bit positions and scaling come from opendbc/dbc/V09-EV-.dbc.
// IMPORTANT: every signal in this DBC is big-endian (Motorola, "le=False").
// opendbc's dbc parser reports the physical LSB bit (in standard MSB-first
// CAN bit numbering) for each signal; the helpers below read Motorola signals
// from that LSB + size.

// ---------- signal helpers (Motorola / big-endian, LSB + size in bits) ----------
// The parser's be_bits[] numbering maps frame bit k to standard position
// be_bits[k] = (k/8)*8 + (7 - k%8). For a big-endian signal, value bit b (0 = LSB)
// lives at be_bits index (be_bits.index(lsb) - b). We resolve be_bits.index(lsb)
// arithmetically: k = (lsb & ~7) + (7 - (lsb & 7)).
static int gwm_get_signal(const CANPacket_t *msg, int lsb, int size) {
  int L = (lsb & ~7) + (7 - (lsb & 7));  // be_bits.index(lsb)
  int ret = 0;
  for (int b = 0; b < size; b++) {
    int k = L - b;                        // be_bits index of value bit b (0 = LSB)
    int p = (k / 8) * 8 + (7 - (k % 8));  // standard bit position = be_bits[k]
    int byte = p / 8;
    int bit = p % 8;                      // MSB-first bit within the byte
    int val = ((msg)->data[byte] >> bit) & 0x1U;
    ret |= (val << b);
  }
  return ret;
}

static int gwm_get_signed_signal(const CANPacket_t *msg, int lsb, int size) {
  int raw = gwm_get_signal(msg, lsb, size);
  int sign_bit = 1 << (size - 1);
  if ((raw & sign_bit) != 0) {
    raw -= (1 << size);
  }
  return raw;
}

// OP 是否接管纵向控制。为 false 时保持原车 ACC(转发原车 ADAS_14),
// 为 true 时屏蔽原车 ADAS_14 由 OP 自己发。默认关闭, 由 init 的 param 设置。
static bool gwm_longitudinal = false;

// ---------- message addresses ----------
#define GWM_ADAS_16_ADDR      412U   // 0x19c  lateral angle command (TX)
#define GWM_ADAS_14_ADDR      390U   // 0x186  longitudinal accel command (TX)
#define GWM_ADAS_3_ADDR       547U   // 0x223  ADAS state / ACC info (RX)
#define GWM_EPS_1_ADDR        870U   // 0x366  measured steering angle (RX)
#define GWM_EPS_2_ADDR        362U   // 0x16a  driver steering torque (RX)
#define GWM_EPS_3_ADDR        360U   // 0x168  EPS target echo / faults (RX)
#define GWM_IBCS_2_A_ADDR     608U   // 0x260  vehicle speed (RX)
#define GWM_VCU_2_A_ADDR      864U   // 0x360  gear / ready (RX)
#define GWM_VCU_21_A_ADDR     374U   // 0x176  brake & accel pedal (RX)

// ---------- angle / accel conversion constants ----------
// ADAS_StrAngleReq: value = deg * 10  (factor 0.1, offset -780)
#define GWM_ANGLE_DEG_TO_CAN  10.0f
// ADAS_LongCtrlTargetAccel: value = (accel_m_s2 + 5) * 20  (factor 0.05, offset -5)
#define GWM_ACCEL_OFFSET      5.0f
#define GWM_ACCEL_FACTOR      20.0f

#define GWM_ANGLE_OFFSET_CAN  ((int)(780.0f * GWM_ANGLE_DEG_TO_CAN))  // 7800

static uint32_t gwm_get_checksum(const CANPacket_t *to_push) {
  // Checksum is at byte 7 (bit 63, 8 bits)
  return (uint32_t)(to_push)->data[GET_LEN(to_push) - 1U];
}

static uint32_t gwm_compute_checksum(const CANPacket_t *to_push) {
  // GWM checksum: sum of bytes 0-6, XOR 0xFF
  int len = GET_LEN(to_push);
  uint8_t checksum = 0U;
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)(to_push)->data[i];
  }
  return checksum ^ 0xFFU;
}

static uint8_t gwm_get_counter(const CANPacket_t *to_push) {
// All monitored GWM messages carry a 4-bit counter, but it is located at
// different physical LSBs depending on the message (big-endian DBC):
//   EPS_1:  lsb 2   (EPS_1_MsgCounter)
//   ADAS_3: low nibble of byte 0 (DBC 3|4@0+ points to the wrong bits;
//           observed IDCU traffic cycles 0..15 in byte 0 low nibble, NOT bits 3-6
//           which the DBC claims. Reading the DBC-declared bits causes
//           wrong_counters to saturate to MAX_WRONG_COUNTERS within 5 frames,
//           making is_msg_valid() fail and forcing controls_allowed = false.
//           See debug-gwm-lat-ctrl-allowed.md for the byte-level evidence.)
//   others: lsb 48  (EPS_2 / EPS_3 / IBCS_2_A / VCU_21_A / ADAS_14 / ADAS_16)
int addr = to_push->addr;
if (addr == GWM_EPS_1_ADDR) {
  return (uint8_t)gwm_get_signal(to_push, 2, 4);
}
if (addr == GWM_ADAS_3_ADDR) {
  // ADAS_3 counter is the low 4 bits of byte 0 (MSB-first bit positions 4-7).
  // Observed IDCU traffic: byte 0 cycles 0x20, 0x21, ..., 0x2F, 0x20, ... -
  // the LOW nibble is the counter (0..15), the HIGH nibble is a constant
  // (0x2 in observed traffic). The DBC entry 3|4@0+ points to the wrong
  // bits and was the original cause of wrong_counters saturating.
  // gwm_get_signal's formula does not handle this position correctly, so
  // we read the low nibble directly.
  return (uint8_t)((to_push)->data[0] & 0xFU);
}
return (uint8_t)gwm_get_signal(to_push, 48, 4);
}

static bool gwm_get_quality_flag_valid(const CANPacket_t *to_push) {
  int addr = to_push->addr;
  bool valid = true;

  if (addr == GWM_EPS_1_ADDR) {
    // EPS_SteeringAngleVD (lsb 34): standard DBC semantics — 1 = valid, 0 = invalid.
    // carstate.py mirrors this (VD=True → use EPS angle; VD=False → fall back to SAS_1).
    // PREVIOUSLY this was inverted (!VD), making the quality_flag permanently false
    // (observed bit 34 is 1 on every real frame) → is_msg_valid() fail →
    // safety_rx_checks_invalid=True → controlsd refuses to enable, even though
    // ADAS_3 counter is now fixed and the angle is being read correctly.
    valid = gwm_get_signal(to_push, 34, 1) != 0U;
  } else if (addr == GWM_EPS_3_ADDR) {
    // EPS_ErrFeedback (lsb 3, 4 bits) must be 0 and target angle within range
    int err_feedback = gwm_get_signal(to_push, 3, 4);
    bool targ_ovr = gwm_get_signal(to_push, 40, 1) != 0U;   // EPS_RecTargAngOvRang
    bool targ_spd = gwm_get_signal(to_push, 41, 1) != 0U;   // EPS_RecTargAngSpdOvRang
    valid = (err_feedback == 0) && !targ_ovr && !targ_spd;
  }

  return valid;
}

static void gwm_rx_hook(const CANPacket_t *to_push) {
  int addr = to_push->addr;
  int bus = to_push->bus;

  if (bus == 0U) {
    // ---- measured steering angle (EPS_1) ----
    if (addr == GWM_EPS_1_ADDR) {
      // EPS_SteeringAngle: raw = (deg+780)*10 (big-endian, signed)
      int raw = gwm_get_signed_signal(to_push, 16, 16);
      // store zero-centered: physical_deg * DEG_TO_CAN
      update_sample(&angle_meas, raw - GWM_ANGLE_OFFSET_CAN);
    }

    // ---- driver steering torque (EPS_2) -> override detection ----
    // NOTE: do NOT reset controls_allowed here. EPS angle control naturally
    // allows driver override (the EPS torque sensor detects driver input and
    // gives way). Resetting controls_allowed would force the user to re-engage
    // stock cruise after every steering intervention, which is not how other
    // openpilot safety models behave. Driver override is handled by the Python
    // layer (controlsd.py -> CS.steeringPressed).
    if (addr == GWM_EPS_2_ADDR) {
      // EPS_2_StrngWhlTorq: raw = (Nm+15)*100, unsigned 12-bit (always >= 0)
      int raw = gwm_get_signal(to_push, 32, 12);
      float driver_torque_nm = (float)raw / 100.0f - 15.0f;   // physical Nm (offset -15)
      update_sample(&torque_driver, (int)(driver_torque_nm * 100.0f));
    }

    // ---- brake & gas pedals (VCU_21_A) ----
    if (addr == GWM_VCU_21_A_ADDR) {
      brake_pressed = gwm_get_signal(to_push, 53, 1) != 0U;   // VCU_EMS_BrkPedalSt_A
      int accel = gwm_get_signal(to_push, 40, 8);  // VCU_EMS_AccPedalActPst_A (raw = pedal%/0.392)
      gas_pressed = accel != 0U;
    }

    // ---- vehicle speed (IBCS_2_A): BCS_VehSpd, factor 0.05625 km/h ----
    if (addr == GWM_IBCS_2_A_ADDR) {
      int veh_spd = gwm_get_signal(to_push, 40, 13);  // BCS_VehSpd, 0..240 km/h
      float speed_ms = (float)veh_spd * 0.05625f / 3.6f;
      vehicle_moving = veh_spd > 0;
      UPDATE_VEHICLE_SPEED(speed_ms);
    }

    // ---- ADAS state (ADAS_3): ACC / TJA / LCC work state ----
    if (addr == GWM_ADAS_3_ADDR) {
      int acc_work = gwm_get_signal(to_push, 27, 4);      // ADAS_ACC_WorkSt (Motorola lsb=27)
      int tja_lcc_work = gwm_get_signal(to_push, 20, 3);  // ADAS_TJA_LCC_Workst (Motorola lsb=20)
      if ((acc_work != 0) || (tja_lcc_work != 0)) {
        controls_allowed = true;
      }
    }
  }

  // ---- ADAS state (ADAS_3) on bus 2 ----
  // ADAS_3 originates on bus 2 (IDCU side). The panda does not self-receive
  // its own forwarded TX copies on STM32 CAN, so rx_hook only sees the
  // original bus-2 reception.
  if (bus == 2U && addr == GWM_ADAS_3_ADDR) {
    int acc_work = gwm_get_signal(to_push, 27, 4);      // ADAS_ACC_WorkSt
    int tja_lcc_work = gwm_get_signal(to_push, 20, 3);  // ADAS_TJA_LCC_Workst
    if ((acc_work != 0) || (tja_lcc_work != 0)) {
      controls_allowed = true;
    }
  }
}

static bool gwm_tx_hook(const CANPacket_t *to_send) {
  static const AngleSteeringLimits GWM_ANGLE_STEERING_LIMITS = {
    .max_angle = 4500,  // 450 deg * 10 CAN units/deg
    .angle_deg_to_can = GWM_ANGLE_DEG_TO_CAN,
    // MUST stay in sync with CarControllerParams.ANGLE_LIMITS in car/gwm/values.py
    // (same units: deg per ADAS_16 message, 50 Hz). Aligned with Tesla's
    // ISO 11270-derived limits: highway up 0.2 / down 0.3 deg per message.
    .angle_rate_up_lookup = {
      {5., 25., 25.},
      {1.2, 0.2, 0.2}
    },
    .angle_rate_down_lookup = {
      {5., 25., 25.},
      {1.5, 0.3, 0.3}
    },
    .enforce_angle_error = false,
    // Upstream default (Tesla/Toyota/Nissan): while steer control is not enabled,
    // the commanded angle must stay near angle_meas instead of being exactly 0.
    // This is what lets apply_std_steer_angle_limits echo the measured wheel angle
    // on idle frames, so the command is continuous across the engage boundary.
    //
    // This was previously set to true as a workaround, together with forcing the
    // idle angle to 0 in carcontroller.py. That pair broke command continuity and
    // forced every engage to start with a step of the full measured angle (observed
    // 12.6 deg and up to 52 deg), which is what pulls the wheel to one side.
    //
    // The reason the workaround was needed no longer applies: angle_meas is fed by
    // gwm_rx_hook from EPS_1 unconditionally, and its bit extraction
    // (gwm_get_signed_signal(to_push, 16, 16)) was verified byte-for-byte against the
    // DBC parse on 3005 real EPS_1 frames (0 mismatches). So the inactive check now
    // has a correct reference to compare against.
    .inactive_angle_is_zero = false,
  };

  // longitudinal accel limits expressed in CAN units of ADAS_LongCtrlTargetAccel
  const LongitudinalLimits GWM_LONG_LIMITS = {
    .max_accel = (int)(GWM_ACCEL_FACTOR * (2.0f + GWM_ACCEL_OFFSET)),    // 140  -> 2.0 m/s^2
    .min_accel = (int)(GWM_ACCEL_FACTOR * (-3.5f + GWM_ACCEL_OFFSET)),   // 30   -> -3.5 m/s^2
    .inactive_accel = (int)(GWM_ACCEL_FACTOR * (0.0f + GWM_ACCEL_OFFSET)),  // 100 -> 0 m/s^2
  };

  bool tx = false;
  int addr = to_send->addr;
  int bus = to_send->bus;

  if (bus == 0U) {
    // ---- lateral angle command (ADAS_16) ----
    if (addr == GWM_ADAS_16_ADDR) {
      tx = true;
      int raw = gwm_get_signed_signal(to_send, 8, 16);  // ADAS_StrAngleReq, raw = (deg+780)*10
      int desired_angle = raw - GWM_ANGLE_OFFSET_CAN;    // physical_deg * DEG_TO_CAN
      bool l2_fun = gwm_get_signal(to_send, 22, 2) != 0U;          // ADAS_L2FunReqSt
      int control_mode = gwm_get_signal(to_send, 52, 4);           // ADAS_ControlMode
      // GWM uses L2FunReqSt=2 + ControlMode=3 to enable angle control
      // (stock IDCU on bus0; cm=1 was observed to trigger EPS_ErrFeedback=4)
      bool steer_control_enabled = l2_fun != 0U;

      if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, GWM_ANGLE_STEERING_LIMITS)) {
        tx = false;
      }

      // No angle control allowed when controls are not allowed
      if (!controls_allowed && steer_control_enabled) {
        tx = false;
      }

      // when not actively steering, force control mode to the inactive value
      if (!steer_control_enabled && (control_mode != 0U)) {
        tx = false;
      }
    }

    // ---- longitudinal accel command (ADAS_14) ----
    if (addr == GWM_ADAS_14_ADDR) {
      tx = true;
      // ADAS_LongCtrlTargetAccel: raw = (accel+5)*20. The value range maps to
      // raw 30..140, so it is an UNSIGNED 8-bit signal (signed would misread 140 as -116).
      int desired_accel = gwm_get_signal(to_send, 0, 8);    // ADAS_LongCtrlTargetAccel
      bool accel_req = gwm_get_signal(to_send, 55, 1) != 0U;       // ADAS_LongCtrlAccelCtrlReq

      // NOTE: the shared longitudinal_accel_checks() in this repo is a debug-modified
      // version that force-sets controls_allowed = true, so we do the range + enable
      // check locally instead to avoid that side effect.
      bool violation = (desired_accel > GWM_LONG_LIMITS.max_accel) || (desired_accel < GWM_LONG_LIMITS.min_accel);
      // only allow a non-inactive accel command when controls are allowed
      if (!controls_allowed && (desired_accel != GWM_LONG_LIMITS.inactive_accel)) {
        violation = true;
      }
      // if accel control is not requested, only the inactive (no-command) value is allowed
      if (!accel_req && (desired_accel != GWM_LONG_LIMITS.inactive_accel)) {
        violation = true;
      }

      if (violation) {
        tx = false;
      }
    }

    // ---- ADAS state (ADAS_3): OP retransmits with TJA_LCC_Workst=2 ----
    // Stock IDCU only sends TJA_LCC_Workst=1 (standby). EPS requires
    // TJA_LCC_Workst=2 (LCC active) to execute ADAS_16 angle commands.
    // Without this, EPS returns ErrFeedback=5 and refuses to steer.
    if (addr == GWM_ADAS_3_ADDR) {
      tx = true;
    }
  }

  return tx;
}

static safety_config gwm_init(uint16_t param) {
  static const CanMsg GWM_TX_MSGS[] = {
    {GWM_ADAS_16_ADDR, 0, 8, .check_relay = false},
    {GWM_ADAS_14_ADDR, 0, 8, .check_relay = false},
    {GWM_ADAS_3_ADDR,  0, 8, .check_relay = false},  // OP sends ADAS_3 with TJA_LCC_Workst=2 to activate LCC
  };

  // GWM checksum: sum(bytes 0-6) ^ 0xFF = byte 7. Validated on 50+ messages.
  static RxCheck gwm_rx_checks[] = {
    // measured steering angle
    {.msg = {{GWM_EPS_1_ADDR, 0, 8, .ignore_checksum = false, .ignore_counter = false,
              .max_counter = 15U, .ignore_quality_flag = false, .frequency = 100U}, {0}, {0}}},
    // driver steering torque
    {.msg = {{GWM_EPS_2_ADDR, 0, 8, .ignore_checksum = false, .ignore_counter = false,
              .max_counter = 15U, .frequency = 100U}, {0}, {0}}},
    // EPS target echo / faults
    // quality_flag DISABLED: EPS_3.EPS_ErrFeedback is observed = 5 at stock idle
    // (NOT a fault - see carstate.py notes). Requiring it == 0 made the RX check
    // permanently invalid -> safetyRxChecksInvalid -> controls blocked.
    {.msg = {{GWM_EPS_3_ADDR, 0, 8, .ignore_checksum = false, .ignore_counter = false,
              .max_counter = 15U, .ignore_quality_flag = true, .frequency = 50U}, {0}, {0}}},
    // vehicle speed
    // NOTE: BCS_VehSpd (bits 40..52) overlaps BCS_2_A_MsgCounter (bits 48..51) in this
    // DBC, so the counter cannot be separated from the speed signal. Counter checks are
    // disabled here; resolve with a real CAN trace / DBC fix before enabling.
    {.msg = {{GWM_IBCS_2_A_ADDR, 0, 8, .ignore_checksum = false, .ignore_counter = true,
              .frequency = 50U}, {0}, {0}}},
    // gear / vehicle ready
    // ignore_checksum: VCU_2_A byte7 is NOT a sum^0xFF checksum (100% mismatch on
    // real bus, 602/602 frames). Forcing the check made RX permanently invalid.
    {.msg = {{GWM_VCU_2_A_ADDR, 0, 8, .ignore_checksum = true, .ignore_counter = true,
              .frequency = 10U}, {0}, {0}}},
    // brake & accel pedals
    {.msg = {{GWM_VCU_21_A_ADDR, 0, 8, .ignore_checksum = false, .ignore_counter = false,
              .max_counter = 15U, .frequency = 50U}, {0}, {0}}},
    // ADAS state (ACC / TJA / LCC work state) — arrives on bus 2 (IDCU side)
    // NOTE: registered at bus 2 because panda does not self-receive forwarded
    // bus2->bus0 copies; the original bus-2 reception must be validated here.
    {.msg = {{GWM_ADAS_3_ADDR, 2, 8, .ignore_checksum = false, .ignore_counter = false,
              .max_counter = 15U, .frequency = 50U}, {0}, {0}}},
  };

  // param bit 0: OP 接管纵向控制 (屏蔽原车 ADAS_14 转发)。默认关闭 -> 原车 ACC。
  const uint16_t GWM_PARAM_LONGITUDINAL = 1U;
  gwm_longitudinal = GET_FLAG(param, GWM_PARAM_LONGITUDINAL);

  safety_config ret;
  SET_TX_MSGS(GWM_TX_MSGS, ret);
  SET_RX_CHECKS(gwm_rx_checks, ret);
  return ret;
}

static bool gwm_fwd_hook(int bus_num, int addr) {
  // 截断型 harness (与 toyota/tesla/subaru 等摄像头车同款模式):
  //   bus0 = 车辆总线(EPS/VCU/IBCS), bus2 = 原车 IDCU/ADAS 侧。
  // 目标框架自动完成 bus0 <-> bus2 的双向转发; 本 hook 只声明哪些
  // 来自 IDCU(bus2) 的控制帧必须被屏蔽, 防止与原车 IDCU 的同一报文冲突。
  // 返回 true = 屏蔽该帧的转发。bus0 方向的消息不做屏蔽。
  bool block_msg = false;

  if (bus_num == 2) {
    // 屏蔽原车横向控制帧 ADAS_16 (OP 始终接管横向)
    if (addr == GWM_ADAS_16_ADDR) {
      block_msg = true;
    }
    // 屏蔽原车 ADAS_3 (OP 自己发, 设置 TJA_LCC_Workst=2 激活 LCC;
    // 原车 IDCU 只发 TJA_LCC_Workst=1 standby, EPS 拒绝执行角度控制)
    if (addr == GWM_ADAS_3_ADDR) {
      block_msg = true;
    }
    // 屏蔽原车纵向控制帧 ADAS_14, 仅当 OP 接管纵向时
    if (gwm_longitudinal && addr == GWM_ADAS_14_ADDR) {
      block_msg = true;
    }
  }

  return block_msg;
}

const safety_hooks gwm_hooks = {
  .init = gwm_init,
  .rx = gwm_rx_hook,
  .tx = gwm_tx_hook,
  .fwd = gwm_fwd_hook,
  .get_checksum = gwm_get_checksum,
  .compute_checksum = gwm_compute_checksum,
  .get_counter = gwm_get_counter,
  .get_quality_flag_valid = gwm_get_quality_flag_valid,
};
