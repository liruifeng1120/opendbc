from opendbc.car import Bus, structs, get_safety_config
from opendbc.car.gwm.carstate import CarState
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.values import CAR, DBC, CarControllerParams
from opendbc.car.interfaces import CarInterfaceBase

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "gwm"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]

    # **** angle control ****
    ret.steerControlType = SteerControlType.angle
    ret.steerActuatorDelay = 0.18   # TODO: tune for this EPS (Tesla uses 0.1)
    ret.steerLimitTimer = 0.8
    ret.steerAtStandstill = True    # angle control cars allow steering at standstill

    # ==== 记录仪模式已关闭：恢复横向控制 ====
    # 2026-08-30 dashcam 录制完成，steerRatio 已标定(24.0)，符号已核对(右正)。
    # ⚠️ 台架测试会污染内存中的 steerRatio（车不动方向转动会学高），上路前务必
    # 重启 OP（重启后 paramsd 会用收紧的上限自动重置被污染值）。
    ret.dashcamOnly = False

    # **** longitudinal ****
    # GWM checksum 已逆向: sum(bytes0-6) ^ 0xFF = byte7
    # 横向控制已可用；纵向控制 (ADAS_14) 暂未启用，保持原车 ACC。
    # 启用纵向时需在 carcontroller.py 恢复 ADAS_14 发送逻辑并配套安全层验证。
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = False
    ret.minEnableSpeed = -1.       # TODO: set real minimum enable speed
    ret.stoppingDecelRate = 0.3
    ret.vEgoStopping = 0.25
    ret.vEgoStarting = 0.25
    ret.longitudinalActuatorDelay = 0.15  # TODO: tune

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, docs: bool) -> structs.CarParamsSP:
    # GWM has no sunnypilot-specific params; return the defaults.
    return ret
