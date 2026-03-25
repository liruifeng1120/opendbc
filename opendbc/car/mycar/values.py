
from dataclasses import dataclass, field
from enum import IntFlag
from opendbc.car import Bus, DbcDict, PlatformConfig, Platforms, CarSpecs
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
# 车辆静态参数配置
class CarControllerParams:
  #允许的转向角度偏移范围不超过预设值
  STEER_DRIVER_ALLOWANCE = 68
  # 车辆线控转向（Steer-by-Wire）系统中，该参数用于调节 ‌驾驶员输入信号的增益系数‌
  STEER_DRIVER_MULTIPLIER = 3
  #驾驶员转向输入的动态调节系
  STEER_DRIVER_FACTOR = 1
  #车辆实际航向与规划路径之间的最大允许横向偏差
  STEER_ERROR_MAX = 40   #50

  ###########################禁止启动调试参数 否则后果自负！
  STANDSTILL_DEBUG= False #静止状态‌调试主扭矩最大值   最大爬升速率  最大下降速率 #True
  VEHICLE_LESS_DEBUG = False  # 无车‌调试
  ###############################横向工作控制

  STEER_ARTIFICIAL=20  #干预方向盘扭矩  20
  ARTIFICIAL_TIME = 5  # 干预方向盘关闭横向时间 秒
  SPEED_STATE=1  #车速启动横向 千米/小时

  ###############################主扭矩可调试参数

  STEER_MAX = 240  #方向盘最大允许扭矩  300
  STEER_DELTA_UP =5  #单位时间内方向盘扭矩允许爬升的速率  20
  STEER_DELTA_DOWN =7   #单位时间内方向盘扭矩允许下降的速率  20


  ############################  横向控制  车身稳定调试参数

  Tuning_steerActuatorDelay = 0.2  # 如果转弯过早则减小，过晚则增加 低速不走直线可以调节不是优先调节  参考值0.2
  Tuning_steerLimitTimer = 0.2   # 定义转向角度限制的持续时间阈值为200ms    参考值0.2
  Tuning_torque_friction = 0.18  # 控制转向系统的摩擦阻尼 开始转动的力矩，使车辆可以在直道上居中, 不贴近两边  最大0.25 低速不走直线优先调节 参考值0.2
  Tuning_torque_latAccelFactor = 2  # 横向加速度因子 使车辆可以在中等速度下通过中等弯道。 值越小车辆在弯道中的转向响应越灵敏   如果friction大于0.25 就调节这个 参考值 2
  Tuning_torque_kp = 1.0  #  快速响应转向偏差，值越大修正越激进，但可能引发超调 此处设为1.0，  参考值1.0
  Tuning_torque_ki = 0.11  #  消除稳态误差（如长期偏向某一侧），值越小越滤波小 修正动作越多  低速不走直线优先调节   参考值0.1


#FD to be added later
class GacSafetyFlags(IntFlag):
  MY_CAR = 0x1

@dataclass
class GacCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  #todo add docs and harness info

@dataclass
class GacPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "my_car"})
  #todo add dbc for other models

class CAR(Platforms):
  MY_CAR = GacPlatformConfig(
    [GacCarDocs("MY CAR")],
    CarSpecs(mass=1785.0, wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

#汽车CAN总线通信的基础类
class CanBus:
  ESC = 0 # ESC总线  电子稳定控制系统总线（Electronic Stability Control）
  MRR = 1 # 雷达总线  毫米波雷达总线（Medium Range Radar）
  MPC = 2 # MPC总线  模型预测控制器总线（gyroscope Predictive Controller）
#CAN总线固件查询配置
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST], #查询制造商软件版本的标准请求指令
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],#接收ECU返回的软件版本信息
      bus=CanBus.ESC,
    ),
  ],
)

DBC = CAR.create_dbc_map()


