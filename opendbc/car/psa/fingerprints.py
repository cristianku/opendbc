from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'212053276', # Peugeot e208 Allure Pack
        b'194504751', # Peugeot e208 GT CC-only
        b'222256113', # Peugeot e208 GT NZ
    ],
  },
  CAR.PSA_PEUGEOT_508: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'200603842', # Peugeot 508 Hybrid
    ],
  },
  CAR.PSA_PEUGEOT_3008: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'xxxxxxx',
    ],
    # DIRECTN - Electronic Power Steering
    (Ecu.eps, 0x7D0, None): [ ],
    # HCU2 - Hybrid Control Unit
    (Ecu.hybrid, 0x6A6, None): [],
    (Ecu.transmission, 0x7E1, None): [ ],
    # MSB - Electronic Brake Booster
    (Ecu.electricBrakeBooster, 0x6B4, None): [],
    # VCU - Vehicle Control Unit
    (Ecu.engine, 0x7E0, None): [ ],
    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        b'085065201906190129', # Peugeot 3008
    ],
  }
}
