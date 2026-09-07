from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness

Ecu = CarParams.Ecu


class CarControllerParams:
  # *** lateral angle control limits (ADAS_16.ADAS_StrAngleReq) ***
  # TODO: verify STEER_ANGLE_MAX against the real EPS limit (DBC allows +/-780 deg)
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    450.0,  # deg, max steering wheel angle we command
    # (speed_bp, rate_limit_up_bp) deg/frame (ADAS_16 sent at 50Hz)
    # Aligned with Tesla's ISO 11270-derived limits (tesla/values.py):
    #   up   ([0., 5., 25.], [2.5, 1.5, 0.2])  -> deg/s: 125 / 75 / 10
    #   down ([0., 5., 25.], [5., 2.0, 0.3])   -> deg/s: 250 / 100 / 15
    # Tesla keeps a 0 m/s breakpoint for low-speed creep; GWM's EPS has not been
    # characterized below 5 m/s so we clamp there instead (up 60 deg/s below 5 m/s,
    # same as Tesla's 5 m/s value). Before this the high-speed up limit was 0.5
    # deg/frame (25 deg/s) = 2.5x Tesla, which contributed to the "wheel yanked at
    # 16-32 deg/s" observed in 00000011--dbc5c96c30--0.
    ([5, 25], [1.2, 0.2]),
    # (speed_bp, rate_limit_down_bp) deg/frame
    ([5, 25], [1.5, 0.3]),
  )

  # torque limits forwarded to EPS (ADAS_16.ADAS_SACMotorTorLimExt*)
  # EPS uses these as an upper bound on the motor torque while executing the angle command
  # Stock IDCU sends ±5.0 Nm. ROAD TEST 00000003 showed the EPS target clamps at
  # ~±8.6 deg and then DIVERGES to the opposite direction when the command exceeds
  # ~±9 deg — the EPS cannot hold larger angles under a ±5 Nm torque cap (internal
  # control winds up). Raise to the DBC maximum (±10.22 Nm) to give the EPS enough
  # authority to reach and hold larger commanded angles.
  SAC_TORQUE_MAX = 10.22   # Nm (DBC max)
  SAC_TORQUE_MIN = -10.22  # Nm (DBC min)

  # Driver override (Nissan-style linear torque-cap scaling). The EPS motor
  # torque cap is scaled DOWN linearly as the driver takes over, so they can
  # always overpower the EPS smoothly instead of fighting a full ±10.22 Nm step.
  # Two takeover triggers are OR-ed together (see carcontroller.update):
  #   1. hand torque  |EPS_2.EPS_2_StrngWhlTorq| > DRIVER_OVERRIDE_TORQUE
  #   2. |measured angle rate| > DRIVER_OVERRIDE_ANGLE_RATE
  # Trigger (2) exists because a no-torque takeover slips through every torque
  # threshold: route 00000018 (2026-09-05) peaked at only 1.42 Nm while the
  # driver cranked the wheel 100+ deg/s — under the old 1.5 Nm-only logic the
  # controller never conceded and gave the EPS full authority the whole time.
  # DRIVER_OVERRIDE_MIN_FRAC floors the cap so the EPS never goes fully limp and
  # still re-engages cleanly once the driver lets go.
  DRIVER_OVERRIDE_TORQUE = 1.5        # Nm, >= threshold controlsd uses for steeringPressed (carstate.STEER_THRESHOLD)
  # deg/s measured wheel speed beyond which the DRIVER is taken to be steering.
  # Must sit well above what the EPS itself produces while executing our command:
  # with the wind-up clamp + rate-limited recovery, autonomous tracking of the
  # sub-0.01 curvature here is only a few deg/s, and even a 5 deg wind-up gap is
  # walked back at the DOWN limit (~75 deg/s low speed). A physical wheel spinning
  # at 40+ deg/s against us is a driver / hand-over, not the EPS doing its job.
  # (Route 00000018's takeovers reached 100+ deg/s at <1.5 Nm — invisible to any
  # torque test but unmistakable on rate.)
  DRIVER_OVERRIDE_ANGLE_RATE = 40.0   # deg/s
  DRIVER_OVERRIDE_MIN_FRAC = 0.2      # floor EPS cap at 20% of max (~2.0 Nm) during takeover
  DRIVER_OVERRIDE_SLOPE = 0.6         # Nm of EPS cap removed per Nm of hand torque above the threshold

  # --- deprecated aliases kept for readability in old call sites ---
  SAC_TORQUE_OVERRIDE_MIN_FRAC = DRIVER_OVERRIDE_MIN_FRAC
  SAC_TORQUE_OVERRIDE_SLOPE = DRIVER_OVERRIDE_SLOPE
  STEER_THRESHOLD = DRIVER_OVERRIDE_TORQUE  # must match carstate.STEER_THRESHOLD

  def __init__(self, CP):
    self.ACCEL_MAX = 2.0    # m/s^2, TODO: confirm with vehicle
    self.ACCEL_MIN = -3.5   # m/s^2, TODO: confirm with vehicle

    # not a torque car, but these are used by the angle rate limiter timing
    self.STEER_DELTA_UP = 10
    self.STEER_DELTA_DOWN = 25


@dataclass
class GwmCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.obd_ii]))


def dbc_dict(pt):
  return {Bus.pt: pt}


# FW query config for GWM. The car uses standard UDS (ISO-TP) on the OBD-II port.
# `extra_ecus` lists the likely diagnostic addresses so the fingerprint routine
# actually queries the car and captures its firmware (used to populate FW_VERSIONS).
# TODO: trim/extend this list once the real responding ECUs are known.
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
    ),
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
    ),
    Request(
      [StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST],
      [StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_RESPONSE],
    ),
  ],
  extra_ecus=[
    (Ecu.engine, 0x7e0, None),        # EMS
    (Ecu.transmission, 0x7e1, None),  # TCU
    (Ecu.abs, 0x7e2, None),           # ESP/ABS
    (Ecu.eps, 0x7e4, None),           # EPS
    (Ecu.fwdCamera, 0x756, None),     # ADAS camera
    (Ecu.fwdRadar, 0x736, None),      # ACC radar
  ],
)


class CAR(Platforms):
  # TODO: rename to the real marketing name once known
  GWM_V09_EV = PlatformConfig(
    [
      GwmCarDocs("GWM V09 EV"),
    ],
    # V09 actual specs (2026-09-02 updated from the vehicle spec sheet).
    # mass 2655 kg / wheelbase 3.17 m. NOTE steerRatio and wheelbase only enter
    # the lateral model as the PRODUCT sR*l (yaw ~ v*steer/(sR*l)): the 19.0 was
    # calibrated by check_steer_ratio.py with WHEELBASE=2.8, so with the real
    # 3.17 m wheelbase the ratio must be rescaled 19.0 * 2.8 / 3.17 = 16.8 to
    # keep the calibrated product (53.2) unchanged. Leaving 19.0 with l=3.17
    # would command 13% more wheel angle for the same curvature.
    # steerRatio: kinematic value = effective ratio at low speed (slip ~ 0).
    #   Dashcam route 00000000--816bd5c13e (EPS_1 angle vs SRS_IMU yaw vs vEgo)
    #   gave speed-segmented effective ratios 18.8 @18-32 km/h .. 28.5 @90-104
    #   km/h with l=2.8 -> with l=3.17: kinematic ~16.6 .. effective ~25.2.
    #   Sign check: 4984/4984 consistent.
    CarSpecs(mass=2655., wheelbase=3.17, steerRatio=16.8, tireStiffnessFactor=1.0),
    dbc_dict('V09-EV-'),
  )


DBC = CAR.create_dbc_map()
