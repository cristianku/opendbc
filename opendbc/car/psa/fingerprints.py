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
        b'xxxxxxx', # Citroen Berlingo
    ],
    ############# PLACEHOLDER ################
    # DIRECTN - Electronic Power Steering
    (Ecu.eps, 0x6B5, None): [
        b'6077GC0817309',
        b'\xbfP\x00\x00\x13j\x07\x06\x15\xb5@\xf5\x03\xff\xff\xff\x00\x02\x00\x00\x01\x944g'
    ],
    # HCU2 - Hybrid Control Unit
    (Ecu.hybrid, 0x6A6, None): [
        b'210306062100',
        b'\xff\xff\x00\x00\r\n\x06\x03!\x03\x01\x12\x01\xff\xff\xff\x00\x02\x00\x00\x02\x94\x86b'
    ],
    # MSB - Electronic Brake Booster
    (Ecu.electricBrakeBooster, 0x6B4, None): [
        b'521021900860',
        b'\xff\xff\x00\x00t\x01\x11\x01!\x01\x040\x15\xff\xff\xff\x00\x02\x00\x00\xfe\x95\x08w'
    ],
    # VCU - Vehicle Control Unit
    (Ecu.engine, 0x6A2, None): [
        b'9210126909',
        b'\xf2i\x00\x00\r\x99\x11\x05\x15\x01!\xb2\x01!\x07#\xfd\xd4S\xe2\x02\x96E '
    ],
    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        b'085065201906190129', # Peugeot 3008
        b'085095200769200218', # Citroen Berlingo 2019 CC
        b'085095700857210527',
        b'\x00\x00\x00\x00\x03\x93!\x08 \x01\xc2\x12\x12\xff\xff\xff\x00\x02\x00\x00\x01\x94'
    ],
    ############### PLACEHOLDER END ##############
  }
}
