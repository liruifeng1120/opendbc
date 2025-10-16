# Compatibility shim: expose CANDefine at opendbc.can.can_define
from opendbc.can.parser import CANDefine

__all__ = ["CANDefine"]
