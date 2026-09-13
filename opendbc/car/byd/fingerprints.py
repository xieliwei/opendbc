""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.byd.values import CAR

Ecu = CarParams.Ecu


FW_VERSIONS = {
  CAR.BYD_ATTO_3: {
    (Ecu.fwdCamera, 0x704, None): [
      b'\x9c\xad\x19\x06\x04\x00',
    ],
    (Ecu.abs, 0x782, None): [
      b'\x9c\x50\x17\x0c\x1b\x01',
    ],
    (Ecu.eps, 0x783, None): [
      b'\x9c\x49\x17\x06\x1e\x00',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'\x75\x53\x18\x04\x14\x23',
    ],
    (Ecu.srs, 0x7f1, None): [
      b'\x9c\x45\x18\x01\x1a\x01',
    ],
    (Ecu.fwdRadar, 0x7f2, None): [
      b'\x9c\xa4\x16\x02\x19\x01',
    ],
  },
}
