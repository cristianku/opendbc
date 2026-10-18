---
name: psa-long-findings-2026-10-17
description: Confirmed facts from the 2026-10-17 traffic-light/roundabout investigation (brake gain regimes, vehicle tracking, DEC, ki=0, no video locally)
metadata:
  type: project
---

Investigation of premature braking at traffic lights/roundabouts (2026-10-17), tooling in
openpilot_scripts/log_analysis/longitudinal_investigation/ (outputs in out/, gitignored).

- Brake gain actually applied, measured from CAN 0x2B6 vs carControl.accel: ~1.0 on routes up to
  0000007d (Sept), ~1.5 (1.55 minus filter) from 000000b8 on, incl. all 14-16 Oct routes
  (openpilot commits fb1c36569, a8b556a07 — not present in the local openpilot checkout).
- Vehicle brake ECU tracks MDD_DESIRED_DECELERATION within ~0.1 m/s^2 (layer F is fine) ->
  with 1.55 the measured decel is ~1.4x the openpilot request; LongControl kiV=[0] (pure FF).
- DEC (DynamicExperimentalControl) was active in every route (build predates sunnypilot 5ec0c0219
  "Sunset DEC", 2026-10-16): e2e candidate only in 'blended'; ~2.4 switches per engaged minute.
- radarUnavailable=True: all leads are vision; MPC STOP_DISTANCE 6 m -> median standstill gap ~5.5 m.
- No fcamera/qcamera anywhere locally (all downloads --no-video): video evidence must be fetched by Cristian.
- Overpass: overpass-api.de needs a User-Agent (406 without), often 502 under load; mail.ru mirror times out.

**Why:** these are expensive to re-derive and drive any future brake-LUT/hysteresis calibration.
**How to apply:** do not re-measure the gain regime; reuse extract.py timelines; treat 1.0-vs-1.55 route
comparisons as confounded by different models (OP10, RDF, RDF V2) and SCC settings.
