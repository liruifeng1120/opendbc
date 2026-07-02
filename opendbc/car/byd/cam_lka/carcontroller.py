import math
import numpy as np
import cereal.messaging as messaging

from opendbc.can.packer import CANPacker

from opendbc.car.byd.cam_lka.bydcan import (
  create_accel_command,
  create_can_steer_command,
  create_lkas_hud,
  create_resume_sequence,
  create_steering_torque_spoof_camera,
  send_buttons,
)
from opendbc.car.byd.values import DBC, CAR, ACCEL_MULT, CANBUS, BYD_ATTO_STYLE_PLATFORMS, BYD_OP_LONG_PLATFORMS
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

try:
  from openpilot.common.swaglog import cloudlog
except ImportError:
  cloudlog = None

STEER_LOWPASS_HZ = 2
STEER_DT = 0.02
MAX_STEER_ANGLE_OFFSET_DEG = 10
# Degrees: warn on HUD when command hits angle safety envelope (meas offset or global max).
STEER_ANGLE_LIMIT_WARN_EPS_DEG = 0.08
LKA_COOLDOWN_MIN_FRAMES = 30
SPOOF_DURATION_FRAMES = 50
SPOOF_CYCLE_FRAMES = 150
RESUME_SEQUENCE_STEP_FRAMES = 5  # 50 ms spacing at 100 Hz
SNG_INITIAL_PRESS_DELAY_FRAMES = 310
SNG_REPEAT_PRESS_DELAY_FRAMES = 110
SNG_WAIT_LOG_INTERVAL_FRAMES = 200  # ~2s at 100Hz controller frame counter


def lowpass_1pole(x, y_prev):
  if y_prev is None:
    return x
  alpha = math.exp(-2.0 * math.pi * STEER_LOWPASS_HZ * STEER_DT)
  return alpha * y_prev + (1.0 - alpha) * x


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])

    self.lka_active = False
    self.last_apply_angle = 0
    self.accel_mult = ACCEL_MULT[CP.carFingerprint]
    self.lka_cooldown = 0
    self.prev_press = False
    self.prev_res_press = False
    self.lka_latched = False
    self.button_send_bus = CANBUS.cam_bus if CP.carFingerprint in BYD_ATTO_STYLE_PLATFORMS else CANBUS.main_bus
    # A stock-like resume tap is sent as a short multi-frame sequence.
    self.resume_sequence_frames = []
    self.resume_sequence_counter = 0
    self.resume_sequence_next_frame = 0
    # SNG state is local now that cam_lka is the only remaining user.
    self.plan_sm = messaging.SubMaster(['longitudinalPlan']) if not CP.openpilotLongitudinalControl else None
    self.sng_next_press_frame = 0
    self.sng_resume_counter = 0
    self.sng_active = False
    self.sng_prev_plan_allows = False
    self.sng_brake_block_logged = False

  def _start_resume_sequence(self):
    sequence_frames, self.resume_sequence_counter = create_resume_sequence(self.packer, self.button_send_bus, self.resume_sequence_counter)
    # Send the first frame immediately and queue the rest to preserve stock-like timing
    self.resume_sequence_frames = sequence_frames[1:]
    self.resume_sequence_next_frame = self.frame + RESUME_SEQUENCE_STEP_FRAMES
    return sequence_frames[0]

  def _log_sng(self, event: str, detail: str):
    if cloudlog is not None:
      cloudlog.warning(f"BYD_SNG {event} {detail}")

  def _plan_fields(self):
    if self.plan_sm is None or not self.plan_sm.valid['longitudinalPlan']:
      return 0.0, True, False, False, -1

    plan = self.plan_sm['longitudinalPlan']
    return (
      float(plan.aTarget),
      bool(plan.shouldStop),
      True,
      bool(plan.hasLead),
      int(plan.longitudinalPlanSource),
    )

  def _plan_allows_resume(self):
    if self.plan_sm is None or not self.plan_sm.valid['longitudinalPlan']:
      return False

    plan = self.plan_sm['longitudinalPlan']
    return plan.aTarget > 0 and not plan.shouldStop

  def _sng_snapshot(self, CS, enabled, resume_pressed, plan_allows_resume):
    a_target, should_stop, plan_valid, has_lead, plan_src = self._plan_fields()
    cruise_hud = int(getattr(CS, 'cruise_hud_state', -1))
    return (
      f"f={self.frame} act={int(self.sng_active)} ego={int(CS.out.standstill)} "
      f"cstk={int(CS.out.cruiseState.standstill)} v={CS.out.vEgoRaw:.2f} "
      f"cruise={int(CS.out.cruiseState.enabled)} hud={cruise_hud} en={int(enabled)} "
      f"plan_ok={int(plan_allows_resume)} pvalid={int(plan_valid)} aT={a_target:.2f} "
      f"stop={int(should_stop)} lead={int(has_lead)} psrc={plan_src} "
      f"gas={int(CS.out.gasPressed)} brk={int(CS.out.brakePressed)} "
      f"rx_res={int(resume_pressed)} set={int(getattr(CS, 'set_btn', -1))} "
      f"cnt={self.sng_resume_counter} next={self.sng_next_press_frame}"
    )

  def _log_sng_press(self, msgs):
    a_target, should_stop, plan_valid, has_lead, plan_src = self._plan_fields()
    dats = ",".join(bytes(msg_dat).hex() for _, msg_dat, _ in msgs)
    addr, _, bus = msgs[0]
    self._log_sng(
      'PRESS',
      f"f={self.frame} n={self.sng_resume_counter} bus={bus} addr=0x{addr:x} "
      f"nframes={len(msgs)} dat={dats} aT={a_target:.2f} stop={int(should_stop)} "
      f"lead={int(has_lead)} psrc={plan_src} pvalid={int(plan_valid)}",
    )

  def _plan_block_reason(self):
    if self.plan_sm is None or not self.plan_sm.valid['longitudinalPlan']:
      return 'plan_invalid'

    plan = self.plan_sm['longitudinalPlan']
    if plan.aTarget <= 0:
      return 'plan_aT0'
    if plan.shouldStop:
      return 'plan_shouldStop'
    return 'plan_unknown'

  def _sng_disarm_reason(self, CS, enabled, plan_allows_resume):
    if not CS.out.standstill:
      return 'no_standstill'
    if not enabled:
      return 'not_enabled'
    if not CS.out.cruiseState.enabled:
      return 'cruise_off'
    if not plan_allows_resume:
      return self._plan_block_reason()
    return 'unknown'

  def _update_sng_sequence(self, enabled, CS, can_sends):
    if self.plan_sm is None:
      return

    # Resume sequence frames follow their own cadence once started.
    if self.frame % 2 != 0:
      return

    if self.resume_sequence_frames:
      if self.frame >= self.resume_sequence_next_frame:
        can_sends.append(self.resume_sequence_frames.pop(0))
        self.resume_sequence_next_frame = self.frame + RESUME_SEQUENCE_STEP_FRAMES
      return

    self.plan_sm.update(0)
    resume_pressed = CS.res_btn
    plan_allows_resume = self._plan_allows_resume()

    if not (CS.out.standstill and enabled and CS.out.cruiseState.enabled and plan_allows_resume):
      if self.sng_active:
        self._log_sng('DISARM', f"{self._sng_disarm_reason(CS, enabled, plan_allows_resume)} {self._sng_snapshot(CS, enabled, resume_pressed, plan_allows_resume)}")
      self.sng_active = False
      self.sng_brake_block_logged = False
      self.sng_prev_plan_allows = plan_allows_resume
      return

    if plan_allows_resume and not self.sng_prev_plan_allows:
      self._log_sng('PLAN_GO', self._sng_snapshot(CS, enabled, resume_pressed, plan_allows_resume))

    if not self.sng_active:
      self.sng_active = True
      self.sng_next_press_frame = self.frame + SNG_INITIAL_PRESS_DELAY_FRAMES
      self.sng_resume_counter = 0
      self.sng_brake_block_logged = False
      self._log_sng('ARM', self._sng_snapshot(CS, enabled, resume_pressed, plan_allows_resume))
      self.sng_prev_plan_allows = plan_allows_resume
      return

    if self.frame % SNG_WAIT_LOG_INTERVAL_FRAMES == 0 and self.frame <= self.sng_next_press_frame:
      self._log_sng('WAIT', self._sng_snapshot(CS, enabled, resume_pressed, plan_allows_resume))

    if CS.out.gasPressed or resume_pressed:
      reason = 'gas' if CS.out.gasPressed else 'manual_res'
      self._log_sng('COOLDOWN', f"{reason} {self._sng_snapshot(CS, enabled, resume_pressed, plan_allows_resume)}")
      self.sng_next_press_frame = max(self.sng_next_press_frame, self.frame + SNG_REPEAT_PRESS_DELAY_FRAMES)
      self.sng_resume_counter = 0
      self.sng_brake_block_logged = False
      self.sng_prev_plan_allows = plan_allows_resume
      return

    self.sng_prev_plan_allows = plan_allows_resume

    if self.frame > self.sng_next_press_frame and CS.out.brakePressed:
      if not self.sng_brake_block_logged:
        self._log_sng('BLOCK_BRAKE', self._sng_snapshot(CS, enabled, resume_pressed, plan_allows_resume))
        self.sng_brake_block_logged = True
      return

    if self.frame > self.sng_next_press_frame:
      self.sng_resume_counter += 1
      # Treat one retry as one complete stock-like resume tap.
      msgs = [self._start_resume_sequence(), *self.resume_sequence_frames]
      can_sends.append(msgs[0])
      self.sng_next_press_frame = self.frame + SNG_REPEAT_PRESS_DELAY_FRAMES
      self.sng_brake_block_logged = False
      self._log_sng_press(msgs)

  def _update_lka_latch_state(self, CS):
    if self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SHARK, CAR.BYD_SEALION7):
      lkas_rising = CS.lkas_rdy_btn and not self.prev_press
      if lkas_rising:
        if not self.lka_latched:
          self.lka_latched = True
        else:
          self.lka_latched = False
          self.lka_cooldown = 0
          self.lka_active = False
      elif (
        self.CP.carFingerprint == CAR.BYD_SHARK
        and CS.res_btn
        and not self.prev_res_press
        and not self.lka_latched
      ):
        # Shark: stock RES (resume) also arms LKA latch; disengage still via LKAS btn or brake.
        self.lka_latched = True
      self.prev_press = CS.lkas_rdy_btn
      self.prev_res_press = CS.res_btn

      if CS.out.brakePressed:
        self.lka_latched = False
        self.lka_cooldown = 0
        self.lka_active = False
      elif self.lka_latched:
        self.lka_active = True
        self.lka_cooldown += 1
    else:
      if CS.lka_on:
        self.lka_cooldown += 1
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False
        self.lka_cooldown = 0

  def _compute_apply_angle(self, CS, actuators, lat_active):
    meas_deg = CS.out.steeringAngleDeg
    if not lat_active:
      return meas_deg, False

    limits = CarControllerParams.ANGLE_LIMITS
    after_lowpass = lowpass_1pole(actuators.steeringAngleDeg, self.last_apply_angle)
    after_std = apply_std_steer_angle_limits(
      after_lowpass,
      self.last_apply_angle,
      CS.out.vEgo,
      meas_deg,
      lat_active,
      limits,
    )
    lo = meas_deg - MAX_STEER_ANGLE_OFFSET_DEG
    hi = meas_deg + MAX_STEER_ANGLE_OFFSET_DEG
    apply_angle = float(np.clip(after_std, lo, hi))
    self.last_apply_angle = apply_angle

    eps = STEER_ANGLE_LIMIT_WARN_EPS_DEG
    meas_clip_limited = abs(after_std - apply_angle) > eps
    angle_max_limited = abs(apply_angle) >= limits.STEER_ANGLE_MAX - eps
    steer_angle_limited = meas_clip_limited or angle_max_limited
    return apply_angle, steer_angle_limited

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    self._update_lka_latch_state(CS)

    lat_active = (self.lka_cooldown > LKA_COOLDOWN_MIN_FRAMES) and enabled and self.lka_active and not CS.out.standstill

    apply_angle = CS.out.steeringAngleDeg
    hand_on_wheel_warning = False
    if (self.frame % 2) == 0:
      apply_angle, steer_angle_limited = self._compute_apply_angle(CS, actuators, lat_active)
      hand_on_wheel_warning = bool(lat_active and steer_angle_limited)
      can_sends.append(
        create_can_steer_command(
          self.packer,
          apply_angle,
          lat_active,
          CS.out.standstill,
          CS.lkas_healthy,
          CS.lkas_rdy_btn or CS.out.brakePressed,
        )
      )
      can_sends.append(
        create_lkas_hud(
          self.packer,
          lat_active,
          CS.lss_state,
          CS.lss_alert,
          CS.tsr,
          CS.HMA,
          CS.pt2,
          CS.pt3,
          CS.pt4,
          CS.pt5,
          CS.lkas_hud_status_passthrough,
          CS.lka_on,
          hand_on_wheel_warning,
        )
      )

      if self.CP.carFingerprint in BYD_OP_LONG_PLATFORMS and self.CP.openpilotLongitudinalControl:
        long_active = CC.enabled and not CS.out.gasPressed
        brake_hold = CS.out.standstill and actuators.accel < 0
        can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, self.accel_mult, brake_hold))

    self._update_sng_sequence(CC.enabled, CS, can_sends)

    if self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SEALION7, CAR.BYD_SHARK):
      cycle_position = self.frame % SPOOF_CYCLE_FRAMES
      spoof_active = cycle_position < SPOOF_DURATION_FRAMES

      if (self.frame % 5) == 0:
        can_sends.append(
          create_steering_torque_spoof_camera(self.packer, lat_active, CS.out.steeringTorque, spoof_active)
        )

    if pcm_cancel_cmd:
      can_sends.append(send_buttons(self.packer, 0, 1, self.button_send_bus))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
