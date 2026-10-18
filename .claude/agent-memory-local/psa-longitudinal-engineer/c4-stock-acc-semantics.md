---
name: c4-stock-acc-semantics
description: Stock ARTIV ACC semantics learned from C4 SpaceTourer dashcam logs (route 00000009--dc7802815d, dongle 378c9bf47606eb28) — 2B6 mux, handover via drag floor, MDD = total decel
metadata:
  type: project
---
Confirmed 2026-10-18 from C4 SpaceTourer stock-ACC rlogs (~/Downloads/last5days/00000009--dc7802815d--10..20, 636 s; NOT the 3008).

- 0x2B6 GEAR_TYPE == COUNTER&1 always; it muxes MIN_TIME_FOR_DESIRED_GEAR: GT=1 -> 6.2 s (upshift), GT=0 -> 3.6 s (downshift) in GMP; 0 in brake/inactive.
- GMP_WHEEL_TORQUE is real wheel Nm: ~= Dyn_CMM.P198 x overall ratio (eta~0.93), tracked within a frame. C4 fit a ~= 1.36e-3*T - 0.09 - 1.53e-4*v^2.
- Stock never requests GMP torque below engine-drag floor (~ -Dyn2_CMM.P017_Com_trqFrc x ratio, -60..-120 Nm); switches to brake (POT=2) when more decel needed; WHEEL_TORQUE_REQUEST=2 (low range) marks the floor around handovers/activation.
- MDD_DESIRED_DECELERATION = total vehicle decel, ESP closes loop: bias ~0 on C4, lag 0.56-0.94 s, ramp <=0.5-1 m/s^3, exit at -0.15/-0.2.
- ACC_STATUS 3 only >=27 km/h; 5 = driver demand > ACC (ACC keeps sending torque); activation is bumpless (torque init to actual).
- 0x452 on bus 0/2 is a different static frame; only bus 1 is the BSI ACC command.

**Why:** reference behaviour for the 3008 radar-spoofing port (psacan/carcontroller), which differs (fixed -0.5 brake entry, negative GMP lookup to -400 Nm, MIN_TIME 6.2 both parities, filter restarts at 0).
**How to apply:** use as stock baseline when diagnosing [[downhill-brake-hunting]] or proposing 2B6 changes; re-verify on 3008 logs before transferring numbers. Scripts lived in a session scratchpad (gone).
