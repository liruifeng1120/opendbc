#!/usr/bin/env python3
from math import exp
from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mycar.carcontroller import CarController
from opendbc.car.mycar.carstate import CarState
from opendbc.car.mycar.values import CAR, CarControllerParams
#配置车型参数
class CarInterface(CarInterfaceBase):
    CarState = CarState
    CarController = CarController

    @staticmethod
    def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams: # type: ignore
      # 汽车名称
        ret.brand = "my car"
        # Panda
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mycar)]
        # 设置雷达不可用状态标志
        ret.radarUnavailable =True

        #禁用高级驾驶辅助功能
        ret.dashcamOnly = candidate not in CAR.MY_CAR
        ret.minEnableSpeed = -1.0
        ret.minSteerSpeed = 0.1 * CV.KPH_TO_MS

        ret.steerActuatorDelay = CarControllerParams.Tuning_steerActuatorDelay
        ret.steerLimitTimer = CarControllerParams.Tuning_steerLimitTimer
        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
        ret.lateralTuning.torque.friction = CarControllerParams.Tuning_torque_friction
        ret.lateralTuning.torque.latAccelFactor = CarControllerParams.Tuning_torque_latAccelFactor
        ret.lateralTuning.torque.kp = CarControllerParams.Tuning_torque_kp
        ret.lateralTuning.torque.ki = CarControllerParams.Tuning_torque_ki
        ret.centerToFront = ret.wheelbase * 0.41
        return ret




