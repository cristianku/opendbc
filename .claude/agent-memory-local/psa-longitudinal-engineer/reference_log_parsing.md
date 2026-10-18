---
name: psa-log-parsing-reference
description: How to parse Cristian's rlogs standalone on this Mac (cereal path, buses, segment timing traps)
metadata:
  type: reference
---

- cereal schema in this sunnypilot checkout: new_openpilot_psa_torque_sunny_testing/openpilot/cereal
  (commaai layout under openpilot/); car.capnp must be staged next to log.capnp (see common.py in
  openpilot_scripts/log_analysis/longitudinal_investigation/). Run with `uv run` (PEP 723 headers).
- Logs live in several trees: comma_logs/, log_analysis/drive_logs/<date>/, log_analysis/*_2026*/ —
  walk with rglob("rlog.zst"); some copies are truncated (000000b8--26 in longitudinal_noise_20261001).
- initData is repeated in every segment with the route-start logMonoTime: do not use it as segment t0.
- Our 0x2B6 is sendcan src=1 (PSA_ADAS_BUS); its echo is can src=129.
- longitudinalPlanSP.deprecated.dec.{state,active} still readable with the post-sunset schema.
- Video sync key: narrowRoadEncodeIdx.segmentNum/segmentId by timestampSof (fcamera, 20 fps).
