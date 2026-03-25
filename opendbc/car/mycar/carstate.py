
from opendbc.can.parser import CANParser
from opendbc.car import Bus,   structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mycar.values import DBC
from decrypted.Decrypted import Decrypted
executor = Decrypted()
class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    executor.decrypt_and_carStateHelp()
    executor.call_function("init",self,CP)
  def update(self, can_parsers) -> structs.CarState:
    return  executor.call_function("update",self, can_parsers)
  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages,cam_messages=executor.call_function("get_messages")
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }

