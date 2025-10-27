from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

# Peugeot ECU list https://github.com/ludwig-v/arduino-psa-diag/blob/master/ECU_LIST.md

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
    # ARTIV - Front Radar ADAS
    (Ecu.fwdRadar, 0x6B6, None): [ ],

    # DIRECTN - Electronic power steering
    (Ecu.eps, 0x6B5, None): [ ],

    # HCU2 - Hybrid Control Unit
    (Ecu.hybrid, 0x6A6, None): [  ],

    # BOITEVIT - Automatic transmission  (EAT6/8)
    (Ecu.transmission, 0x6A9, None): [ ],

    # FREINEBB
    (Ecu.electricBrakeBooster, 0x5D0, None): [],

    # INJ - Engine (VCU)
    (Ecu.engine, 0x6A8, None): [ ],

    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        # b'085065201906190129', # Peugeot 3008
    ],
  }
}
