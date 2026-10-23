"""Passive OEM ARTIV lead telemetry for PSA training data.

Only decoded CAN values are considered here. Never synthesize a radar track:
the 0x2F6/0x4F6 lead summaries do not carry lateral position or object class.
This module never transmits CAN and never participates in vehicle actuation.
"""
from dataclasses import dataclass
import math


MAX_SAMPLE_AGE_NS = 300_000_000
MAX_PAIR_SKEW_NS = 150_000_000
MAX_ARTIV_DISTANCE_M = 253.0


@dataclass(frozen=True)
class StockRadarLeadObservation:
  distance_m: float = 0.0
  relative_speed_mps: float = 0.0
  distance_valid: bool = False
  relative_speed_valid: bool = False
  measurement_time_nanos: int = 0


def _fresh(timestamp: int, now_nanos: int) -> bool:
  return timestamp > 0 and now_nanos >= timestamp and now_nanos - timestamp <= MAX_SAMPLE_AGE_NS


def decode_stock_artiv_lead(
  lead_2f6: dict, timestamp_2f6: int,
  lead_4f6: dict, timestamp_4f6: int,
  now_nanos: int,
) -> StockRadarLeadObservation:
  """Return only fresh OEM ARTIV measurements; missing values stay invalid.

  0x2F6 contains TARGET_DETECTED and INTER_VEHICLE_DISTANCE, while 0x4F6
  contains a distance summary and RELATIVE_SPEED. The signals may represent
  separate updates, so motion is valid only when they describe a consistent
  target within a short time window. Neither message is a full radar point.
  """
  if not _fresh(timestamp_2f6, now_nanos) or not lead_2f6.get("TARGET_DETECTED"):
    return StockRadarLeadObservation()

  distance = float(lead_2f6.get("INTER_VEHICLE_DISTANCE", 0.0))
  if not math.isfinite(distance) or not 0.0 < distance < 254.0:
    return StockRadarLeadObservation()

  observation = StockRadarLeadObservation(
    distance_m=distance,
    distance_valid=True,
    measurement_time_nanos=timestamp_2f6,
  )

  if not _fresh(timestamp_4f6, now_nanos) or abs(timestamp_4f6 - timestamp_2f6) > MAX_PAIR_SKEW_NS:
    return observation
  if not lead_4f6.get("TARGET_DETECTED") or int(lead_4f6.get("ARTIV_SENSOR_STATE", -1)) != 2:
    return observation

  motion_distance = float(lead_4f6.get("DISTANCE_GAP", 0.0))
  speed = float(lead_4f6.get("RELATIVE_SPEED", 0.0))
  if not (math.isfinite(motion_distance) and 0.0 < motion_distance < MAX_ARTIV_DISTANCE_M
          and math.isfinite(speed) and -70.0 <= speed <= 70.0):
    return observation
  if abs(distance - motion_distance) > max(4.0, distance * 0.15):
    return observation

  return StockRadarLeadObservation(
    distance_m=distance,
    relative_speed_mps=speed,
    distance_valid=True,
    relative_speed_valid=True,
    measurement_time_nanos=timestamp_2f6,
  )
