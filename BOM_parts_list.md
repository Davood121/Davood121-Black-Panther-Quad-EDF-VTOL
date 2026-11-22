# Black Panther 50 mm Quad-EDF VTOL — Real-World Parts & BOM

## 1. Propulsion & Thrust-Vectoring (Per Pod × 4)
| Qty | Component | Recommended Spec / Model | Weight (ea) | Notes |
| :---: | :--- | :--- | :--- | :--- |
| 4 | **50 mm 11-Blade EDF (4S)** | **FMS 50 mm V2 2627-KV4500** (~1,086 g @ 32.9 A) or **Powerfun 50 mm 4300KV** (~950 g @ ~40 A) | ~76–77 g | PA66+GF / PC housing; total static thrust **~3.8 – 4.3 kgf** |
| 4 | **Brushless ESC** | **40A – 50A 4S BLHeli_32 / AM32** (DShot600) | ~25 g | Use 45A–50A for thermal headroom (~132–160 A total peak) |
| 8 | **Metal-Gear Micro Servo** | **EMAX ES08MA II** (1.8 kg·cm @ 6 V) or **MG90S Metal Gear** (2.2 kg·cm @ 6 V) | ~12–13.4 g | 2 per pod (pitch + yaw); avoid plastic SG90s in flight |

## 2. Flight Control, Power & Radio
| Qty | Component | Recommended Spec / Model | Weight | Notes |
| :---: | :--- | :--- | :--- | :--- |
| 1 | **Flight Controller** | **Matek H743-Wing V3** (STM32H743, 480 MHz, 2 MB Flash) | ~30 g | 13 PWM outputs (drives 4 ESCs + 8 servos = 12 channels) with ArduPilot Lua/TVC mixing |
| 1 | **External Servo UBEC** | **6 V 5 A – 8 A Switching BEC** | ~15 g | Dedicated clean power for the 8 vectoring servos |
| 1 | **LiPo Battery** | **4S 3000 – 4000 mAh 60C–100C** (XT60 / XT90) | ~330–430 g | High-C pack required for 130–160 A peak EDF draw |
| 1 | **RC Receiver + GPS** | **ELRS 2.4 GHz RX** + **M10 GPS/Compass** | ~25 g | Enables stabilized hover, AltHold, and Loiter |

## 3. Frame, Gimbal & Mechanical Hardware
- **Gimbal Rings & Fuselage:** 3D-printed in **PA12-CF (Carbon-Nylon)** or **PETG-CF** (100% infill on gimbal rings & pivot bosses).
- **Arms, Skids & Servo Brackets:** **2 mm – 3 mm 3K Carbon Fiber plate** (CNC or hand-cut) + **8 mm / 10 mm CF tubes** for landing skids.
- **Gimbal Bearings & Shafts:** **8× MR63ZZ (3×6×2.5 mm)** or **MR84ZZ (4×8×3 mm)** flanged ball bearings + **3 mm / 4 mm hardened steel dowel pins** for zero-slop pivots.
- **Drive Gears:** **4× 14T + 4× 16T Mod 0.5 (or Mod 0.8) brass/steel spur gears** matching your servo spline / 3–5 mm shaft.
- **Fasteners:** **M3 × 10 mm / 16 mm** stainless socket-head cap screws + nyloc nuts + medium threadlocker.

## 4. Estimated Weight & Thrust-to-Weight Budget
- **Propulsion + Servos + ESCs:** ~515 g
- **Avionics + Battery (4S 3300 mAh):** ~430 g
- **Frame + Skids + Hardware:** ~500–600 g
- **Estimated All-Up Weight (AUW):** **~1.45 – 1.55 kg**
- **Total Static Thrust:** **~3.8 – 4.3 kgf** → **~2.5 : 1 Thrust-to-Weight Ratio** (hovers around ~45–55% throttle).
