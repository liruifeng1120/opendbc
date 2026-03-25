from opendbc.car.interfaces import CarControllerBase
from decrypted.Decrypted import Decrypted

executor = Decrypted()
class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    executor.decrypt_and_carcontrollerHelp( )
    self.instance = executor.get_class_instance("CarController", dbc_names)
  def update(self, CC, CC_SP, CS, now_nanos):
    return self.instance.update(CC, CS)

