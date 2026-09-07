import numpy as np

from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.gwm.values import CarControllerParams
from opendbc.can import CANPacker

SteerControlType = structs.CarParams.SteerControlType

# ADAS_16 is sent at 50 Hz, i.e. one ADAS_16 frame every 2nd controller frame
# (frame % 2 == 0 below).
#
# L2 engage is driven by the ADAS_L2FunReqSt STATE TRANSITION 1 (Ready) -> 2
# (Active), NOT by how long Ready is held. The EPS must first observe Ready=1 —
# walk through the Ready state — before it accepts Active=2; skipping Ready
# entirely (0 -> 2) makes the EPS refuse to execute at all (see data below). So
# the controller always emits exactly one Ready frame then flips to Active on the
# very next frame: the minimal 1->2 edge, no dwell and no timer.
#
# ROAD DATA — route 00000018 (2026-09-05, commit d52c26231, 3/3 engagements,
# 15-25 km/h) sent 0 -> 2 directly (no Ready state at all):
#   EPS_ErrFeedback   0 -> 4 after 141 / 144 / 163 ms, then 4 for 95% of L2=2 frames
#   EPS_PstOverlaySt  0 -> 3 (never 1, never 2)   [3 = request rejected / fault]
#   EPS_SACMotorTorAct exactly 0.000 Nm for all 13588 EPS_3 frames of the route
#   EPS_TargetAngle   follows the command but the wheel never moves:
#                     |command - measured| pinned at 5.00 deg (= WINDUP_MAX_ERROR_DEG)
#
# Every engagement that walked through Ready executed correctly (err=0, the wheel
# followed the command, EPS motor torque went non-zero):
#   00000011--dbc5c96c30 (1a3b539b1), 84 km/h — PstOverlaySt 0->1->2, err 0
#     for all 1889 EPS_3 frames, SACMotorTorAct down to -4.00 Nm.
#   00000015--60a1299f38 (02ff0b1e6), 23 km/h (same low speed as today) —
#     PstOverlaySt 0->1->2, err 0 until the driver fought the wheel.
# Both held Ready for a dwell (1.0 s and 0.2 s respectively), but the data only
# proves Ready must appear, not that it must dwell. We emit the minimum: one Ready
# frame then Active. If the EPS still rejects (err=4 / PstOverlaySt stuck at 3)
# with a single Ready frame, then it does need Ready to dwell — lengthen the Ready
# hold (e.g. 0.2 s, matching 00000015) and re-check PstOverlaySt walks 0->1->2.
# The command is pinned to the measured wheel while inactive, so no wind-up is
# accumulated during whatever Ready duration we end up using.

# Anti wind-up: the rate limiter integrates the gap between the model angle and the
# wheel, so any window where we send commands that the EPS does not execute (torque
# limit, driver holding the wheel, EPS fault, ...) grows that gap until the EPS
# dumps it into the wheel all at once (observed: 23 deg). The command is never
# allowed to drift further than this from the measured wheel angle while active,
# so the worst case stays a small step instead of a yank.
WINDUP_MAX_ERROR_DEG = 5.0

# After the EPS rejects an L2 request (ErrFeedback 4/5 or PstOverlaySt==3) we drop
# L2. EPS_3 clears the reject a few frames after L2=0, so without a cooldown the
# controller would immediately re-enter Ready->Active and could flap if the EPS
# keeps refusing. Hold the disengage for this many controller frames (~0.4s) so a
# genuinely broken handshake gives one clean drop instead of a buzz.
L2_REJECT_HOLD_FRAMES = 40


def gwm_checksum(data: bytes) -> int:
  """GWM checksum: sum of bytes 0-6, XOR 0xFF."""
  return (sum(data[:7]) ^ 0xFF) & 0xFF


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.params = CarControllerParams(self.CP)
    self.last_angle = 0.0
    self.packer = CANPacker(dbc_names[Bus.pt])
    # GWM messages carry a 4-bit rolling counter that must increment by 1 each
    # transmission. Using self.frame % 16 is wrong because we only send ADAS_3/
    # ADAS_16 every 2 controller frames, making the counter jump by 2. The EPS
    # observes this as an invalid sequence and responds with EPS_ErrFeedback=5.
    self.adas3_counter = 0
    self.adas16_counter = 0
    # L2 handshake state: whether the single Ready (1) frame of this engagement has
    # already been sent. Cleared on disengage so the next engage re-emits Ready.
    self.l2_ready_done = False
    # Frames remaining in the EPS-reject hold (see update / L2_REJECT_HOLD_FRAMES).
    self.l2_reject_hold = 0
    # NOTE: no return-to-center (removed 2026-09-03). Disengage is a true
    # disengage: l2_fun/ControlMode/AssistPra go to 0 immediately and the
    # command follows the measured angle (EPS sees zero error -> zero torque,
    # the wheel stays wherever the driver has it, including mid-corner). Keeping
    # the EPS engaged to ramp the angle to 0 caused the "yanked during handover"
    # in 00000015--60a1299f38--: in a 30-deg corner the controller pulled the
    # command to 0 at 75 deg/s with ±10 Nm, fighting the driver.

  def _rate_limited_towards(self, current: float, target: float, v_ego: float) -> float:
    """Move `current` one step towards `target`, capped by the DOWN angle rate limit.

    deg/frame, same units as ANGLE_LIMITS. ADAS_16 goes out at 50 Hz, so the low
    speed entry (1.5 deg/frame) is 75 deg/s. Used wherever the command has to
    travel back towards the measured wheel (wind-up recovery) so that the EPS is
    never handed a step it can dump into the wheel at full torque.
    """
    bp, rv = self.params.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN
    rate_lim = float(np.interp(v_ego, bp, rv))
    return float(np.clip(target, current - rate_lim, current + rate_lim))

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    # Angle control requires cruise (ACC). Disengage is a TRUE disengage: no
    # return-to-center, no lingering L2/CM/AssistPra. l2_fun, control_mode and
    # AssistPraSwitchReq follow steer_signals_active directly so the EPS drops
    # out of L2 angle mode the moment CC.latActive falls (or cruise closes),
    # and the command follows the measured wheel angle (apply_std_steer_angle_limits
    # returns it when not active -> EPS sees zero error -> zero torque, the wheel
    # stays wherever the driver put it, including mid-corner).
    # EPS active-reject (ErrFeedback 4/5 or PstOverlaySt==3 for 3+ frames): the EPS
    # has refused our L2 request and is NOT executing the angle command. Continuing
    # to hold L2=2 + full ±SAC torque just winds the rate limiter up against a wheel
    # that will never move (observed: command pinned at meas+5.00 deg all engagement
    # while the model demanded +37 deg — route 00000018). Drop the L2 request so the
    # command re-pins to the measured angle (zero error -> zero torque) instead of
    # fighting an EPS that already said no.
    eps_rejected = getattr(CS, 'eps_active_reject', False)
    # Latch the disengage so a freshly-cleared reject doesn't immediately re-engage.
    if CC.latActive:
      if eps_rejected or self.l2_reject_hold > 0:
        # new rejection -> start the hold; a hold already running just counts down
        if eps_rejected and self.l2_reject_hold == 0:
          self.l2_reject_hold = L2_REJECT_HOLD_FRAMES
        self.l2_reject_hold -= 1
        l2_held = False
      else:
        l2_held = True
    else:
      self.l2_reject_hold = 0
      l2_held = True

    angle_active = CC.latActive and CS.out.cruiseState.enabled and l2_held
    steer_signals_active = CC.latActive and l2_held

    can_sends = []

    # ****** lateral: angle control (ADAS_16, 50Hz) ******
    if self.CP.steerControlType == SteerControlType.angle and self.frame % 2 == 0:
      # --- ADAS_3: copy stock + set TJA_LCC_Workst=1 while steering (same as before) ---
      if CS.adas3_signals:
        adas3_values = dict(CS.adas3_signals)
        if steer_signals_active:
          adas3_values["ADAS_TJA_LCC_Workst"] = 1
        adas3_values["ADAS_3_MsgCounter"] = self.adas3_counter
        addr3, dat3, bus3 = self.packer.make_can_msg("ADAS_3", 0, adas3_values)
        dat3 = bytearray(dat3)
        dat3[7] = gwm_checksum(dat3)
        can_sends.append((addr3, bytes(dat3), bus3))
        self.adas3_counter = (self.adas3_counter + 1) % 16

      # --- ADAS_L2FunReqSt: 0 -> 1 (Ready) -> 2 (Active) ---
      # The EPS is driven by the STATE TRANSITION 1 -> 2, not by how long Ready is
      # held: it must first observe Ready=1 (go through the Ready state) before it
      # accepts Active=2. Skipping Ready entirely (0 -> 2) made the EPS refuse to
      # execute (ErrFeedback=4 + zero torque, see the engage-handshake note at the
      # top of this file). We therefore always emit one Ready frame then flip
      # straight to Active on the next frame — no dwell, no timer; just the 1->2
      # edge. Disengage is immediate (no reverse handshake on the way out).
      if steer_signals_active:
        if not self.l2_ready_done:
          l2_fun = 1          # first active frame: emit Ready so the EPS walks through the state
          self.l2_ready_done = True
        else:
          l2_fun = 2          # subsequent frames: Active
      else:
        self.l2_ready_done = False
        l2_fun = 0
      control_mode = 3 if steer_signals_active else 0

      # --- ADAS_16: angle command ---
      # Standard angle-control call (matches Tesla / Toyota / Nissan). When
      # angle_active=False, apply_std_steer_angle_limits returns the measured
      # steering wheel angle, so the command stays continuous across the engage
      # boundary: idle frames echo the measured angle (EPS sees zero error ->
      # zero torque), and the first active frame starts from exactly where the
      # wheel already is. No return-to-center: in a corner the wheel just stays
      # where it is on disengage.
      self.last_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.last_angle, CS.out.vEgoRaw,
                                                     CS.out.steeringAngleDeg, angle_active, self.params.ANGLE_LIMITS)
      # Wind-up guard, only while active. Catches every "we command, EPS does
      # not move" window (torque limit, driver holding the wheel, EPS fault):
      # the command may not drift further than WINDUP_MAX_ERROR_DEG from the real
      # wheel, so the worst case the EPS can dump is one bounded step instead of
      # a 20+ deg yank.
      # The recovery is RATE LIMITED, not the hard min/max clamp it used to be.
      # With a hard clamp the allowed band [meas-5, meas+5] moves together with
      # the wheel, so a driver turning at 300 deg/s drags the command along at
      # 300 deg/s with no rate limit at all — precisely the step this guard
      # exists to prevent. Walking back through the DOWN rate limit keeps
      # |d(command)/dt| bounded in every case.
      if angle_active:
        meas = CS.out.steeringAngleDeg
        upper = meas + WINDUP_MAX_ERROR_DEG
        lower = meas - WINDUP_MAX_ERROR_DEG
        if self.last_angle > upper:
          self.last_angle = self._rate_limited_towards(self.last_angle, upper, CS.out.vEgoRaw)
        elif self.last_angle < lower:
          self.last_angle = self._rate_limited_towards(self.last_angle, lower, CS.out.vEgoRaw)

      # --- SAC motor torque cap: Nissan-style linear scaling on driver takeover ---
      # Driver is "holding/taking over" if either
      #   (1) hand torque > DRIVER_OVERRIDE_TORQUE (steeringTorque on EPS_2), or
      #   (2) |measured angle rate| > DRIVER_OVERRIDE_ANGLE_RATE even with low
      #       torque (EPS_SteeringAngleSpd is a magnitude-only signal, so we use
      #       the signed carState steeringRateDeg).
      # A takeover that produces no torque — today's route peaked at only 1.42 Nm
      # while the driver cranked the wheel >100 deg/s — is exactly what neither
      # torque threshold nor controlsd's steeringPressed (1.5 Nm) ever sees. With
      # a rising angle-rate the driver is clearly steering themselves, so cut the
      # EPS motor torque cap to the floor. Floor = DRIVER_OVERRIDE_MIN_FRAC of max
      # so the EPS never goes fully limp and still tracks once the driver lets go.
      if steer_signals_active:
        taking_over = CS.out.steeringPressed or \
                      abs(CS.out.steeringRateDeg) > self.params.DRIVER_OVERRIDE_ANGLE_RATE
        if not taking_over:
          sac_max = self.params.SAC_TORQUE_MAX
        else:
          excess = max(0.0, abs(CS.out.steeringTorque) - self.params.DRIVER_OVERRIDE_TORQUE)
          sac_max = max(self.params.SAC_TORQUE_MAX * self.params.DRIVER_OVERRIDE_MIN_FRAC,
                        self.params.SAC_TORQUE_MAX - self.params.DRIVER_OVERRIDE_SLOPE * excess)
      else:
        sac_max = 0.0
      sac_min = -sac_max

      values = {
        "ADAS_StrAngleReq": self.last_angle,
        # Request EPS to switch to LKA assist mode while we steer.
        "ADAS_AssistPraSwitchReq": 1 if steer_signals_active else 0,
        "ADAS_L2FunReqSt": l2_fun,
        "ADAS_SACMotorTorLimExtMax": sac_max,
        "ADAS_SACMotorTorLimExtMin": sac_min,
        "ADAS_ControlMode": control_mode,
        "ADAS_16_MsgCounter": self.adas16_counter,
        "ADAS_16_Checksum": 0,  # placeholder, patched after pack
      }
      addr, dat, bus = self.packer.make_can_msg("ADAS_16", 0, values)
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      can_sends.append((addr, bytes(dat), bus))
      self.adas16_counter = (self.adas16_counter + 1) % 16

    # ****** output bookkeeping ******
    new_actuators = actuators.as_builder()
    new_actuators.torque = 0.0
    new_actuators.steeringAngleDeg = self.last_angle

    self.frame += 1
    return new_actuators, can_sends
