from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.psa.values import DBC


def _create_psa_radar_can_parser(CP):
  messages = [("HS2_DYN_MDD_ETAT_2F6", 20)]
  # return CANParser("psa_aee2010_r3", messages, Bus.radar)
  return CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = _create_psa_radar_can_parser(CP)
    self.track_id = 0

  def update(self, can_strings):
    vls = self.rcp.update(can_strings)
    ret = structs.RadarData()

    sig = self.rcp.vl["HS2_DYN_MDD_ETAT_2F6"]

    if sig["TARGET_DETECTED"]:
      dRel = float(sig["INTER_VEHICLE_DISTANCE"])
      yRel = 0.0  # non disponibile nel DBC, radar centrale
      vRel = 0.0  # mancante, se PSA non fornisce RangeRate

      self.pts[0] = structs.RadarData.RadarPoint(
        dRel=dRel, yRel=yRel, vRel=vRel,
        aRel=float('nan'), yvRel=float('nan'),
        measured=True, trackId=0
      )

    ret.points = list(self.pts.values())
    return ret
