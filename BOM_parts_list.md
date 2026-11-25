# Black Panther Quad Tilt-Rotor — Bill of Materials (`3S` A2212/10T + 8045)

| Qty | Component | Specification | Notes |
|---:|---|---|---|
| 4 | Brushless Motor | `A2212/10T 1400KV` outrunner | `16 × 19 mm` M3 mount, `3.17 mm` shaft + collet prop adapter |
| 4 | Propeller | `8045` (`8 × 4.5 in`, `203.2 mm`, 2× CW + 2× CCW) | Buy injection-molded / glass- or carbon-nylon (do not 3D print) |
| 4 | ESC | `30A` BLHeli_S / BLHeli_32 (`2S–4S`) | One per motor |
| 1 | Battery | **`3S` 2200–3000 mAh `35C+` LiPo (`11.1 V`)** | **Do not use 4S or 6S** with `1400KV + 8045` |
| 8 | Tilt Servos | `MG90S` metal-gear micro servo (or `≥3.5 kg·cm` digital) | 4× Yaw + 4× Pitch (`Pitch_Servo_Mount` included in STL) |
| 1 | Flight Controller + UBEC | `30.5 × 30.5 mm` FC + `5V/6V 5A+` UBEC | ArduPilot / Betaflight custom tilt mixer |
| 4 | Drive Shaft | `5 mm × 26 mm` steel or CF rod | `Gimbal_Drive_Shaft` |
| 12 | Pivot Pins | `3 mm × 8–10 mm` steel dowel pins | `Inner_Pivot_Pins` & `Outer_Pivot_Pins` |
| 4 | Pitch Linkage | `1.5 mm × 22 mm` steel pushrod wire | Connects `Servo_Pitch_MG90S` to `Innermost_Gimbal_Ring` |
| 1 lot | Fasteners | `M3` socket-head bolts + locknuts, `M2` servo screws | Structural & motor mounting |
| 1 set | 3D-Printed Frame | `print_stl/black_panther_a2212_all_printable.stl` | Print in **PETG / ABS / PA-CF**, `30%` gyroid infill, `4` walls |
