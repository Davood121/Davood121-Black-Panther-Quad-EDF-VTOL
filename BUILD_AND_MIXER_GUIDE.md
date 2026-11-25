# Black Panther A2212/10T 1400KV + 8045 Tilt-Rotor Build & Mixer Guide

## 1. 3D Printing Guide (`print_stl/`)
- **Full Drone Plate:** `print_stl/black_panther_a2212_all_printable.stl` (**34 parts**, laid flat on `Z = 0` including `4× Pitch_Servo_Mount` and split flat-laid `Panther_Landing_Skids`).
- **1-Pod Bench Test Plate:** `print_stl/black_panther_a2212_1pod_test.stl` (**7 parts**).
- **Slicer Instructions:**
  1. Open the STL in Bambu Studio / PrusaSlicer / Cura and click **"Split to Objects"** + **"Auto-Arrange"** for your printer's bed.
  2. **Material:** **PETG, ABS, or Carbon-Fiber Nylon (PA-CF)** — do *not* use brittle PLA for gimbal rings or motor cradles.
  3. **Infill & Walls:** `4` perimeters, `30%` gyroid infill (keeps structural airframe mass `~600 g` for `AUW ≤ 1.35–1.55 kg`).

---

## 2. Off-the-Shelf Hardware BOM (`3S` Standardized)
- **Motors (4×):** `A2212/10T 1400KV` brushless outrunners (`16 × 19 mm` M3 mount, `3.17 mm` shaft + collet prop adapter)
- **Propellers (4×):** `8045` (`8 × 4.5 in`, `203.2 mm`) glass-nylon or carbon-nylon props (`2× CW`, `2× CCW` — *buy molded, do not FDM-print*)
- **ESCs (4×):** `30A` BLHeli_S / BLHeli_32 (`2S–4S`)
- **Battery (1×):** **`3S` 2200–3000 mAh `35C+` LiPo (`11.1 V`)** *(do NOT use 4S or 6S with `1400KV + 8045`)*
- **Servos (8×):** `MG90S` metal-gear micro servos (or `≥3.5 kg·cm` digital metal-gear equivalents; 4× Yaw + 4× Pitch)
- **Shafts & Fasteners:**
  - `4× 5 mm × 26 mm` steel or carbon-fiber drive shafts (`Gimbal_Drive_Shaft`)
  - `12× 3 mm × 8–10 mm` steel pivot pins (`Inner_Pivot_Pins`, `Outer_Pivot_Pins`)
  - `4× 1.5 mm` steel pushrod linkages (`Pitch_Pushrod`, `~22 mm`)
  - `M3` socket-head screws & locknuts + `M2` servo mounting screws
- **Avionics:** `30.5 × 30.5 mm` Flight Controller (e.g., Pixhawk/Matek/SpeedyBee running ArduPilot or Betaflight) + `5V/6V 5A+` UBEC for the 8 servos.

---

## 3. Flight Controller Mixer Setup (Tilt-Rotor Modes)
- **Option A — Simpler 1-Axis Tilt (4 Servos, Recommended First Flight):**
  - Lock `Innermost_Gimbal_Ring` neutral and drive only the 4 `Yaw_MG90S` servos (`±20°` limit).
  - In **ArduPilot (`Q_ENABLE = 0` custom servo tilt)** or **Betaflight (`SERVO_TILT` / custom `mmix` + `smix`)**, map differential motor RPM to roll/pitch/yaw hover stabilization and map a forward-flight stick/aux channel to tilt all 4 pods symmetrically (`±20°`).
- **Option B — Full 2-Axis 6-DOF Vectoring (8 Servos):**
  - Requires a custom 12-to-6-DOF control allocator (separating fast motor-RPM attitude stabilization from slower `±20°` gimbal lateral/longitudinal thrust vectoring).
  - Always bench-test 1 pod (`black_panther_a2212_1pod_test.stl`) with `±20°` servo sweeps at `50–100%` throttle on a thrust scale before flying.
