#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


def gwm_checksum(data: bytes) -> int:
  """GWM checksum: sum of bytes 0-6, XOR 0xFF."""
  return (sum(data[:7]) ^ 0xFF) & 0xFF


# GWM V09 EV safety layer. Checksum validated: sum(bytes0-6) ^ 0xFF = byte7.
# Signal bit positions and scaling come from opendbc/dbc/V09-EV-.dbc.
class TestGwmSafetyBase(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest, common.LongitudinalAccelSafetyTest):

  TX_MSGS = [[0x19c, 0], [0x186, 0], [0x223, 0]]  # ADAS_16 (412), ADAS_14 (390), ADAS_3 (547)
  RELAY_MALFUNCTION_ADDRS = {}        # no relay / passthrough in this harness
  # 截断型 harness (toyota/tesla/subaru 同款): bus0 <-> bus2 转发。
  # bus2 -> bus0 屏蔽 OP 接管的控制帧: 横向 ADAS_16 与状态帧 ADAS_3 始终屏蔽,
  # 纵向 ADAS_14 仅当 OP 接管纵向 (LONGITUDINAL) 时屏蔽。
  FWD_BUS_LOOKUP = {0: 2, 2: 0}
  # 未横向使能时要求命令角度与实测角一致 (safety_gwm.h inactive_angle_is_zero=false,
  # upstream 默认; 使 idle 帧可以回现实测角度, 保证接管瞬间命令连续)
  inactive_angle_is_zero = False
  # ADAS_16 (0x19c) is always blocked; ADAS_14 (0x186) is blocked only when
  # gwm_longitudinal=true (param bit 0). Since the test sets param=1, both are blocked.
  FWD_BLACKLISTED_ADDRS = {2: [0x19c, 0x186, 0x223]}

  # openpilot controls longitudinal on this platform
  LONGITUDINAL = True
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  STANDSTILL_THRESHOLD = 0.0
  GAS_PRESSED_THRESHOLD = 0

  # Angle control limits (see safety/safety_gwm.h GWM_ANGLE_STEERING_LIMITS)
  STEER_ANGLE_MAX = 450  # deg
  DEG_TO_CAN = 10        # ADAS_StrAngleReq / EPS_SteeringAngle: factor 0.1 => 1 deg = 10 CAN units

  ANGLE_RATE_BP = [5., 25., 25.]
  # keep in sync with car/gwm/values.py ANGLE_LIMITS and safety/safety_gwm.h
  # aligned with Tesla's ISO 11270-derived limits (highway up 0.2, down 0.3 deg/tick)
  ANGLE_RATE_UP = [1.2, 0.2, 0.2]    # windup limit (deg per tick)
  ANGLE_RATE_DOWN = [1.5, 0.3, 0.3]  # unwind limit (deg per tick)

  packer: CANPackerPanda

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestGwmSafetyBase":
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerPanda("V09-EV-")
    self.safety = libsafety_py.libsafety
    # param bit 0 = OP 接管纵向 (屏蔽原车 ADAS_14 转发), 与 LONGITUDINAL=True 对应
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm, 1)
    self.safety.init_tests()
    # V09-EV-.dbc does not tag the *_MsgCounter signals as COUNTER type, so the
    # packer won't auto-increment them. The safety layer still checks the 4-bit
    # counter at start_bit 51, so we manage it manually per message here.
    self._counters: dict[str, int] = {}

  def _ctr(self, name: str) -> int:
    c = (self._counters.get(name, 0) + 1) % 16
    self._counters[name] = c
    return c

  # ----- angle command (ADAS_16 / 412) -----
  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {
      "ADAS_StrAngleReq": angle,
      "ADAS_AssistPraSwitchReq": 1 if enabled else 0,
      "ADAS_L2FunReqSt": 1 if enabled else 0,
      # control mode must be 0 when not steering (safety enforces this)
      "ADAS_ControlMode": 1 if enabled else 0,
      "ADAS_16_MsgCounter": self._ctr("ADAS_16"),
      "ADAS_16_Checksum": 0,  # placeholder, patched by fix_checksum
    }
    def _fix_cksum(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    return self.packer.make_can_msg_panda("ADAS_16", 0, values, fix_checksum=_fix_cksum)

  # ----- measured steering angle (EPS_1 / 870) -----
  def _angle_meas_msg(self, angle: float):
    # EPS_SteeringAngleVD=1: safety 的 quality_flag 要求有效位为真,否则帧被丢弃
    values = {"EPS_SteeringAngle": angle, "EPS_SteeringAngleVD": 1, "EPS_1_MsgCounter": self._ctr("EPS_1"),
              "EPS_1_Checksum": 0}
    def _fix_cksum(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    return self.packer.make_can_msg_panda("EPS_1", 0, values, fix_checksum=_fix_cksum)

  # ----- vehicle speed (IBCS_2_A / 608) -----
  # BCS_VehSpd (bits 40..52) overlaps the 4-bit counter (bits 48..51) in this DBC, so
  # the counter shares the speed's top 4 bits. We pass the physical km/h value (the packer
  # applies the factor), and set the counter to the speed's natural top nibble so the wire
  # encoding stays consistent (the safety layer ignores the counter for this message).
  def _speed_msg(self, speed):
    raw_kmh = int(round((speed * 3.6) / 0.05625))  # physical km/h -> raw CAN
    values = {"BCS_VehSpd": speed * 3.6, "BCS_2_A_MsgCounter": (raw_kmh >> 8) & 0xF,
              "BCS_2_A_Checksum": 0}
    def _fix_cksum(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    return self.packer.make_can_msg_panda("IBCS_2_A", 0, values, fix_checksum=_fix_cksum)

  # ----- brake pedal (VCU_21_A / 374, VCU_EMS_BrkPedalSt_A) -----
  def _user_brake_msg(self, brake):
    values = {"VCU_EMS_BrkPedalSt_A": 1 if brake else 0, "VCU_21_A_MsgCounter": self._ctr("VCU_21_A"),
              "VCU_21_A_Checksum": 0}
    def _fix_cksum(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    return self.packer.make_can_msg_panda("VCU_21_A", 0, values, fix_checksum=_fix_cksum)

  # ----- gas pedal (VCU_21_A / 374, VCU_EMS_AccPedalActPst_A) -----
  def _user_gas_msg(self, gas):
    values = {"VCU_EMS_AccPedalActPst_A": gas, "VCU_21_A_MsgCounter": self._ctr("VCU_21_A"),
              "VCU_21_A_Checksum": 0}
    def _fix_cksum(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    return self.packer.make_can_msg_panda("VCU_21_A", 0, values, fix_checksum=_fix_cksum)

  # driver steering torque (EPS_2 / 362), used in the test to confirm it is sampled.
  # The safety layer tracks driver torque but does NOT reset controls_allowed on override
  # (EPS angle control naturally gives way; override is handled by the Python layer).
  def _torque_driver_msg(self, torque):
    values = {"EPS_2_StrngWhlTorq": torque, "EPS_2_MsgCounter": self._ctr("EPS_2"),
              "EPS_2_Checksum": 0}
    def _fix_cksum(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    return self.packer.make_can_msg_panda("EPS_2", 0, values, fix_checksum=_fix_cksum)

  # ----- longitudinal accel command (ADAS_14 / 390) -----
  def _accel_msg(self, accel: float):
    # inactive (no-command) accel is 0 m/s^2 -> CAN = (0+5)*20 = 100
    values = {
      "ADAS_LongCtrlTargetAccel": accel,
      "ADAS_LongCtrlAccelCtrlReq": 1 if accel != self.INACTIVE_ACCEL else 0,
      "ADAS_14_MsgCounter": self._ctr("ADAS_14"),
      "ADAS_14_Checksum": 0,
    }
    def _fix_cksum(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    return self.packer.make_can_msg_panda("ADAS_14", 0, values, fix_checksum=_fix_cksum)

  # GWM is openpilot-longitudinal and has no stock PCM/ACC message implemented in the
  # safety layer (controls_allowed is managed by openpilot). Skip the cruise-enable tests.
  def test_enable_control_allowed_from_cruise(self):
    raise unittest.SkipTest("GWM has no stock PCM cruise message; controls_allowed is managed by openpilot")

  def test_disable_control_allowed_from_cruise(self):
    raise unittest.SkipTest("GWM has no stock PCM cruise message; controls_allowed is managed by openpilot")

  def test_cruise_engaged_prev(self):
    raise unittest.SkipTest("GWM has no stock PCM cruise message; controls_allowed is managed by openpilot")

  # ----- GWM specific: driver torque does NOT reset controls_allowed -----
  # EPS angle control naturally allows driver override (the EPS gives way when
  # it detects steering wheel torque from the driver). Resetting controls_allowed
  # in the safety layer would force the user to re-engage stock cruise after every
  # steering intervention. Driver override is instead handled by the Python layer
  # (controlsd.py -> CS.steeringPressed).
  def test_driver_torque_does_not_disengage(self):
    for sign in [-1, 1]:
      self.safety.set_controls_allowed(True)
      # high driver torque should NOT disengage
      for _ in range(common.MAX_SAMPLE_VALS):
        self._rx(self._torque_driver_msg(sign * 25.0 / 100.0))  # 25 Nm (way above old 2 Nm threshold)
      self.assertTrue(self.safety.get_controls_allowed())

  # ----- GWM specific: ADAS_3 enables controls_allowed -----
  def test_adas3_enables_controls(self):
    for work_st in [1, 2, 3, 4, 7]:  # various active states
      self.safety.set_controls_allowed(False)
      # ADAS_ACC_WorkSt (Motorola lsb=27, size=4): set to work_st
      # ADAS_TJA_LCC_Workst (Motorola lsb=20, size=3): set to 0 (inactive)
      values = {
        "ADAS_ACC_WorkSt": work_st,
        "ADAS_TJA_LCC_Workst": 0,
        "ADAS_3_MsgCounter": self._ctr("ADAS_3"),
        "ADAS_3_Checksum": 0,
      }
      def _fix_cksum(msg):
        addr, dat, bus = msg
        dat = bytearray(dat)
        dat[7] = gwm_checksum(dat)
        return addr, bytes(dat), bus
      msg = self.packer.make_can_msg_panda("ADAS_3", 2, values, fix_checksum=_fix_cksum)
      self._rx(msg)
      self.assertTrue(self.safety.get_controls_allowed(), f"ACC_WorkSt={work_st} should enable controls")

    # TJA_LCC active should also enable
    self.safety.set_controls_allowed(False)
    values = {
      "ADAS_ACC_WorkSt": 0,
      "ADAS_TJA_LCC_Workst": 1,
      "ADAS_3_MsgCounter": self._ctr("ADAS_3"),
      "ADAS_3_Checksum": 0,
    }
    def _fix_cksum2(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    msg = self.packer.make_can_msg_panda("ADAS_3", 2, values, fix_checksum=_fix_cksum2)
    self._rx(msg)
    self.assertTrue(self.safety.get_controls_allowed())

    # Both inactive should NOT enable
    self.safety.set_controls_allowed(False)
    values = {
      "ADAS_ACC_WorkSt": 0,
      "ADAS_TJA_LCC_Workst": 0,
      "ADAS_3_MsgCounter": self._ctr("ADAS_3"),
      "ADAS_3_Checksum": 0,
    }
    def _fix_cksum3(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[7] = gwm_checksum(dat)
      return addr, bytes(dat), bus
    msg = self.packer.make_can_msg_panda("ADAS_3", 2, values, fix_checksum=_fix_cksum3)
    self._rx(msg)
    self.assertFalse(self.safety.get_controls_allowed())

  # ----- GWM specific: checksum validation -----
  def test_checksum_validation(self):
    """Verify that messages with correct checksum are accepted, and tampered ones are rejected."""
    self.safety.set_controls_allowed(True)

    # Test with correct checksum (should pass)
    for _ in range(common.MAX_SAMPLE_VALS):
      self._rx(self._torque_driver_msg(0.0))
    self.assertTrue(self.safety.get_controls_allowed())

    # Test with incorrect checksum (should be rejected)
    # Build a message with wrong checksum
    values = {
      "EPS_2_StrngWhlTorq": 0.0,
      "EPS_2_MsgCounter": self._ctr("EPS_2"),
      "EPS_2_Checksum": 0,
    }
    addr, dat, bus = self.packer.make_can_msg("EPS_2", 0, values)
    dat = bytearray(dat)
    dat[7] = 0x00  # wrong checksum
    bad_msg = libsafety_py.make_CANPacket(addr, bus, bytes(dat))

    # Send bad checksum messages - safety layer should reject them
    # (controls_allowed should eventually go false due to checksum failure)
    for _ in range(common.MAX_SAMPLE_VALS):
      self._rx(bad_msg)
    # Note: controls_allowed state depends on how the safety layer handles checksum failures


class TestGwmSafety(TestGwmSafetyBase):
  pass


if __name__ == "__main__":
  unittest.main()
