import numpy as np

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import CAR, DBC

SteerControlType = structs.CarParams.SteerControlType
GearShifter = structs.CarState.GearShifter
ButtonType = structs.CarState.ButtonEvent.Type

# steering wheel torque above this (Nm) is treated as driver override
STEER_THRESHOLD = 1.5  # Nm, TODO: confirm with vehicle

# Fake vehicle speed injection for testing lateral control at standstill/low speed.
# When > 0, overrides the measured vEgo/vEgoRaw with a constant so controlsd's
# standstill check passes and lateral control can be requested. Set to 0 to disable.
FAKE_SPEED_KPH = 0.0


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    # try to read the gear value enumeration if the DBC provides one
    self.gear_values = can_define.dv.get("VCU_2_A", {}).get("VCU_GearLvl_A", None)

    # Cache of original ADAS_3 signal values from the stock IDCU.
    # carcontroller copies these and overrides TJA_LCC_Workst=2 to activate LCC,
    # since the stock IDCU only sends TJA_LCC_Workst=1 (standby) and EPS requires
    # LCC active state to execute ADAS_16 angle commands.
    self.adas3_signals = {}

    # Forced-activation latch driven by GW_CS_2_A (0x190) gear lever signal.
    # GW_CDC_6_A (CDC_ADAS_ACCSwitchReq) is not transmitted by this vehicle, and
    # ADAS_3.ACC_WorkSt is a speed-dependent status flag rather than a driver
    # button. OP is force-started when the lever reports D gear
    # (CS_GearShiftLeverPstmech==5) while driving faster than 15 km/h, and stays
    # latched until the driver presses the brake or speed drops to <= 15 km/h.
    self.op_active = False
    self.mech_5_cnt = 0

    # EPS active-reject latch counter (see update(): EPS_ErrFeedback 4/5 or
    # EPS_PstOverlaySt 3 for 3+ consecutive frames -> eps_active_reject).
    self._eps_reject_cnt = 0

    # Previous steering angle, used to derive the sign of the steering rate.
    # EPS_SteeringAngleSpd is an UNSIGNED magnitude-only signal (0..1016 deg/s,
    # DBC 29|8@0+ factor 4), so it carries no direction. openpilot expects a
    # signed right-positive rate (negative=left, positive=right), so we attach
    # the sign of the measured angle change to the raw magnitude.
    self.prev_steering_angle = None

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    ret = structs.CarState()

    # *** steering angle / rate (feedback from EPS) ***
    # GWM EPS reports right-positive angle, same sign as ADAS_StrAngleReq command
    # (verified from log 0000002b: command +16 deg -> EPS_1 +14..18 deg, and the
    # original IMU yaw rate is consistent with the kinematics only under this sign).
    # The previous negation here inverted the control feedback and caused the
    # runaway angle command ("steering response too large") + steerSaturated alert.
    # NOTE: EPS_SteeringAngleSpd is an UNSIGNED signal (0..1016 deg/s) with no
    # direction info, so it must NOT be negated.
    ret.steeringAngleDeg = cp.vl["EPS_1"]["EPS_SteeringAngle"]
    # EPS_SteeringAngleSpd is unsigned magnitude only (0..1016 deg/s). Attach the
    # sign of the measured angle change so the rate matches the angle convention
    # (negative = turning left, positive = turning right). At standstill the raw
    # magnitude is ~0 so the sign doesn't matter.
    if self.prev_steering_angle is not None:
      angle_delta = ret.steeringAngleDeg - self.prev_steering_angle
      if angle_delta < 0:
        ret.steeringRateDeg = -cp.vl["EPS_1"]["EPS_SteeringAngleSpd"]
      else:
        ret.steeringRateDeg = cp.vl["EPS_1"]["EPS_SteeringAngleSpd"]
    else:
      ret.steeringRateDeg = cp.vl["EPS_1"]["EPS_SteeringAngleSpd"]
    self.prev_steering_angle = ret.steeringAngleDeg
    # SAS is the redundant angle source; use it if EPS angle is invalid
    if not cp.vl["EPS_1"]["EPS_SteeringAngleVD"]:
      ret.steeringAngleDeg = cp.vl["EPS_SAS_1"]["SAS_SteeringAngle"]

    # *** original-car IMU (GWM_SRS_IMU_1_A, SRS airbag module) ***
    # SRS_IMU_YawRate: deg/s, left-positive (openpilot yawRate convention, positive
    # = left). Convert to rad/s, gated by valid + calibrated flags. Feeds the
    # locationd kalman filter as the gyro observation (carState-IMU port).
    imu = cp.vl["GWM_SRS_IMU_1_A"]
    if imu["SRS_IMU_YawRateVD"] and imu["SRS_IMU_YawSnsCalibst"]:
      ret.yawRate = float(np.deg2rad(imu["SRS_IMU_YawRate"]))

    # *** driver steering torque (override detection) ***
    ret.steeringTorque = cp.vl["EPS_2"]["EPS_2_StrngWhlTorq"]
    ret.steeringTorqueEps = cp.vl["EPS_3"]["EPS_SACMotorTorAct"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # *** EPS faults ***
    # The real EPS fault flag is EPS_PAS_EpasFailed (EPS_2 bit 0).
    # Other "feedback/status" signals are NOT faults on this vehicle:
    #   - EPS_3.EPS_ErrFeedback: observed = 6 at stock idle (no DTC on cluster)
    #   - EPS_2.EPS_PAS_AbortFeedback: observed = 7 at stock idle (no DTC)
    # Treating these as faults caused a false "LKAS故障" alert + beep at startup.
    # Temporary fault: EPS_3 target-angle out-of-range bits (observed = 0 at idle).
    ret.steerFaultTemporary = bool(cp.vl["EPS_3"]["EPS_RecTargAngOvRang"]) or \
                              bool(cp.vl["EPS_3"]["EPS_RecTargAngSpdOvRang"])
    ret.steerFaultPermanent = bool(cp.vl["EPS_2"]["EPS_PAS_EpasFailed"])

    # --- EPS active-reject detector (carcontroller drives a hard disengage) ---
    # When the EPS refuses an L2 request it latches:
    #   EPS_ErrFeedback = 4  (mode/handshake error) — 0 on accept (00000011), and
    #                         a stock-idle EPS_3 already reads err=6 even with no
    #                         request (see note above), so treat only {4,5} as reject.
    #   EPS_PstOverlaySt = 3  (request rejected / fault) — accepted L2 drives it
    #                         through 1 (Ready) and 2 (Active), never straight to 3.
    # Exposed as self.eps_active_reject (not ret.steerFault*): a rejection is not a
    # "fault" we want controlsd to sound an alert for — the controller just drops
    # the L2 request and re-pins the command to the measured angle. Exposed while
    # latched for 4 frames so the controller sees it across its own message cadence.
    self.eps_active_reject = False
    _err = int(cp.vl["EPS_3"]["EPS_ErrFeedback"])
    _ovl = int(cp.vl["EPS_3"]["EPS_PstOverlaySt"])
    _reject = (_err in (4, 5)) or (_ovl == 3)
    if _reject:
      self._eps_reject_cnt += 1
    else:
      self._eps_reject_cnt = 0
    if self._eps_reject_cnt >= 3:
      self.eps_active_reject = True

    # *** speed ***
    # Wheel speeds are split across two messages: FL/FR in IBCS_9_A, RL/RR in IBCS_10_A
    self.parse_wheel_speeds(ret,
      cp.vl["IBCS_9_A"]["BCS_FLWheelSpd"],
      cp.vl["IBCS_9_A"]["BCS_FRWheelSpd"],
      cp.vl["IBCS_10_A"]["BCS_RLWheelSpd"],
      cp.vl["IBCS_10_A"]["BCS_RRWheelSpd"],
    )
    ret.vEgoCluster = ret.vEgo
    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    # Fake speed injection: override vEgo so controlsd's standstill check passes
    # even when the vehicle is stationary. This enables lateral control requests
    # without needing to drive at speed. The KF state is left untouched so that
    # disabling FAKE_SPEED_KPH (=0) restores real speed tracking immediately.
    if FAKE_SPEED_KPH > 0:
      fake_ms = FAKE_SPEED_KPH * CV.KPH_TO_MS
      ret.vEgo = fake_ms
      ret.vEgoRaw = fake_ms
      ret.aEgo = 0.0
      ret.standstill = False

    # *** brakes / gear ***
    ret.brakePressed = bool(cp.vl["IBCS_2_A"]["BCS_BrkLightOn"]) or \
                       bool(cp.vl["VCU_21_A"]["VCU_EMS_BrkPedalSt_A"])
    # VCU_CrntGearLvl_A is in VCU_9_A message (not VCU_2_A!)
    # VCU_GearLvl_A in VCU_2_A is always 0 and incorrect
    try:
      can_gear = int(cp.vl["VCU_9_A"]["VCU_CrntGearLvl_A"])
    except Exception:
      can_gear = 0
    # Gear encoding: 0=Park, 1=Drive, 2=Neutral, 3=Reverse (GWM specific)
    gear_map = {0: 'park', 1: 'drive', 2: 'neutral', 3: 'reverse'}
    ret.gearShifter = self.parse_gear_shifter(gear_map.get(can_gear))

    # *** blinkers ***
    ret.leftBlinker = bool(cp.vl["GW_BDC_BCM_2_A"]["BCM_LeftTurnLampSt"])
    ret.rightBlinker = bool(cp.vl["GW_BDC_BCM_2_A"]["BCM_RightTurnLampSt"])

    # *** doors / seatbelt ***
    ret.doorOpen = any([cp.vl["GW_BDC_BCM_2_A"]["BCM_DriverDoorAjarSt"],
                        cp.vl["GW_BDC_BCM_2_A"]["BCM_PsngrDoorAjarSt"],
                        cp.vl["GW_BDC_BCM_2_A"]["BCM_RLDoorAjarSt"],
                        cp.vl["GW_BDC_BCM_2_A"]["BCM_RRDoorAjarSt"]])
    ret.seatbeltUnlatched = cp.vl["VCU_SRS_A"]["VCU_SRS_DriverSeatBeltSt"] != 0

    # *** stock ACC / cruise state (ADAS_3) ***
    # ADAS_3 is on bus2 (IDCU side). fwd_hook blocks its forwarding to bus0,
    # so we parse it directly from bus2. OP retransmits a modified copy on bus0
    # with TJA_LCC_Workst=2 to activate LCC for EPS angle control.
    cp_adas3 = can_parsers.get(Bus.radar)
    adas3_vl = cp_adas3.vl["ADAS_3"] if cp_adas3 and cp_adas3.vl["ADAS_3"] else cp.vl["ADAS_3"]

    # TODO: confirm which ADAS_ACC_WorkSt values mean "active"
    # ACC_WorkSt: 1=standby(ACC off), 2=activating, 3=active, 5=override.
    # Use >=2 so standby(1) is treated as off, matching the stock system.
    acc_work = adas3_vl.get("ADAS_ACC_WorkSt", 0)
    ret.cruiseState.speed = adas3_vl["ADAS_ACC_TargetSpeedSet"] * CV.KPH_TO_MS

    # Cache all ADAS_3 signal values for carcontroller to copy & retransmit
    self.adas3_signals = dict(adas3_vl)

    # *** OP forced activation via GW_CS_2_A gear lever (0x190) ***
    # GW_CDC_6_A (containing CDC_ADAS_ACCSwitchReq) is defined in DBC but this
    # vehicle does NOT transmit it — 0x80-0x8A range is empty in all logs.
    # ADAS_3.ACC_WorkSt is NOT a button either: it follows vehicle speed
    # (flips 1<->2/3 around 13-16 km/h) and is a status flag, not a driver action.
    # Force-start OP when the gear lever reports D (CS_GearShiftLeverPstmech==5)
    # while driving faster than 15 km/h. The activation latches: it stays active
    # until the driver presses the brake or speed drops to <= 15 km/h. Note the
    # mech bit flips back to 1 when OP sends l2 (feedback loop), which is why we
    # latch on a 3-frame confirm instead of requiring mech==5 continuously.
    #
    # sunnypilot engagement mapping: this vehicle has no stock cruise button, so
    # we expose the latch edges through the standard button interface — a rising
    # op_active edge sets ret.buttonEnable (=> buttonEnable event -> engage), and
    # a falling op_active edge emits a cancel button event (=> disengage).
    mech = int(cp.vl["GW_CS_2_A"]["CS_GearShiftLeverPstmech"])
    speed_ok = ret.vEgo > 15. * CV.KPH_TO_MS
    prev_active = self.op_active
    if not self.op_active:
      # only count toward activation while latched-off; any non-D or slow frame resets
      if speed_ok and mech == 5:
        self.mech_5_cnt += 1
      else:
        self.mech_5_cnt = 0
      if self.mech_5_cnt >= 3:
        self.op_active = True
        ret.buttonEnable = True   # engage OP (rising edge)
    else:
      self.mech_5_cnt = 0
      if ret.brakePressed or not speed_ok:
        self.op_active = False
    if not self.op_active and prev_active:
      # OP just latched off this frame -> emit a cancel button edge for disengage
      ret.buttonEvents.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel))
    ret.cruiseState.enabled = self.op_active
    ret.cruiseState.available = self.op_active

    # *** angle-control command echo (for debugging / verification) ***
    # EPS_TargetAngle is the angle the EPS last received; compare against measured
    if self.CP.steerControlType == SteerControlType.angle:
      self.eps_target_angle = cp.vl["EPS_3"]["EPS_TargetAngle"]
      self.assist_pra_switch = bool(cp.vl["ADAS_16"]["ADAS_AssistPraSwitchReq"])

    return ret, structs.CarStateSP()

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    messages = [
      ("EPS_1", 100),            # steering angle + rate
      ("EPS_2", 100),            # steering wheel torque
      ("EPS_3", 100),            # target angle echo + EPS faults
      ("EPS_SAS_1", 100),        # redundant SAS angle
      ("GWM_SRS_IMU_1_A", 50),   # original-car IMU: yaw rate (for locationd)
      ("IBCS_2_A", 50),          # brake light, vehicle speed
      ("IBCS_9_A", 50),           # FL/FR wheel speeds
      ("IBCS_10_A", 50),          # RL/RR wheel speeds
      ("VCU_2_A", 10),           # gear (old, always 0)
      ("VCU_9_A", 10),           # gear (VCU_CrntGearLvl_A)
      ("VCU_21_A", 50),          # brake pedal
      ("GW_BDC_BCM_2_A", 10),    # doors, blinkers
      ("VCU_SRS_A", 10),         # seatbelt
      ("GW_CS_2_A", 100),        # gear lever: CS_GearShiftLeverPstmech (0x190, D=5)
      # NOTE: GW_CDC_6_A is in the DBC but this vehicle doesn't transmit it
      # (0x80-0x8A is empty in all logs). buttonEnable is now derived from
      # GW_CS_2_A gear lever D gear + speed (see update() above). Use float('nan')
      # in case it ever shows up — don't let its absence mark the bus as invalid.
      ("GW_CDC_6_A", float('nan')),  # steering wheel buttons (not on this car)
      # ADAS_16 is our own TX command echo (unreliable for alive-checking).
      ("ADAS_16", float('nan')), # our own lateral command echo
    ]
    parsers = {Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, 0)}
    # ADAS_3 is on bus2 (IDCU side). fwd_hook blocks its forwarding to bus0
    # so OP can retransmit a modified copy (TJA_LCC_Workst=2). Parse directly
    # from bus2 to get the stock IDCU's signal values for copying.
    parsers[Bus.radar] = CANParser(DBC[CP.carFingerprint][Bus.pt], [("ADAS_3", float('nan'))], 2)
    return parsers
