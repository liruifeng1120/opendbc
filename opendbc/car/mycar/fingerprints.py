# ruff: noqa: E501
from opendbc.car.structs import CarParams
from opendbc.car.mycar.values import CAR
Ecu = CarParams.Ecu
#车辆指纹
# Todo: could fingerprints for song dmi be able to combine?
FINGERPRINTS = {
   CAR.MY_CAR: [{
     354: 8, 355: 8, 362: 8, 386: 8, 412: 8, 430: 8, 608: 8, 612: 8, 616: 8, 618: 8, 620: 8, 621: 8, 626: 8, 652: 8,
     660: 8, 662: 8, 676: 8, 699: 8, 700: 8, 806: 8, 814: 8, 825: 8, 830: 8, 835: 8, 836: 8, 839: 8, 849: 8, 858: 8,
     870: 8, 876: 8, 885: 8, 903: 8, 911: 8, 914: 8, 919: 8, 921: 8, 931: 8, 939: 8, 940: 8, 943: 8, 945: 8, 950: 8,
     967: 8, 1300: 8, 1314: 8, 1323: 8, 1325: 8, 1337: 8, 1359: 8
   }]
}
#Todo: Get a byd VDS to see how fw could be queried. Currently added just for preventing ruffs error.

#FW_VERSIONS: dict[str, dict[tuple, list[bytes]]] = {}

FW_VERSIONS = {
  CAR.MY_CAR: {
    (Ecu.eps, 0x366, None): [  #：电动助力转向系统
      b'DUMMYDATA',
    ],
  },
}

