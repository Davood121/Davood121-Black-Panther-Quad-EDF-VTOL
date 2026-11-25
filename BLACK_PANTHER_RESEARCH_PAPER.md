# Propulsion Architecture Trade-Off and Preliminary Aerodynamic-Structural Sizing of the "Black Panther" Tilt-Rotor Quadcopter: 50 mm Ducted EDF vs. A2212/10T 1400KV Open Propeller

**Author:** Shaik Davood  
**Project Workspace:** `c:\Users\shaik\Desktop\new drone desigen`  
**Target Venue Format:** AIAA Regional Student Conference / University Capstone Technical Paper *(see Section VII for Bench/Flight Validation Checklist)*

---

## Abstract

Small-scale vertical takeoff and landing (VTOL) multirotors using thrust-vectoring gimbals offer decoupled 6-degree-of-freedom (6-DOF) pose control and agile horizontal translation without pitching the airframe. This paper presents a comparative propulsion, packaging, and preliminary computational fluid dynamics (CFD) trade study for the **"Black Panther" quadcopter** (`551.54 mm` diagonal wheelbase), evaluating a transition from four `50 mm` 11-blade Electric Ducted Fans (EDFs) to four `A2212/10T 1400KV` brushless outrunners driving `8045` (`203.2 mm`) two-blade twisted propellers on `3S` LiPo power. Parametric 3D CAD assemblies were constructed in FreeCAD 1.1.4 (`black_panther_quad_edf.FCStd` and `black_panther_a2212_quad.FCStd`), and steady Reynolds-Averaged Navier-Stokes (RANS, $k$-$\omega$ SST) actuator-disk simulations were performed in OpenFOAM-12 (`65,952` cells for the bellmouth EDF pod; `22,342` cells for the `A2212 + 8045` pod). Expanding the swept rotor area by **18.9×** (`17.09 cm²` EDF annulus to `322.27 cm²` open disk) reduces disk loading by **25–40×** (`~21.7 kg/m²` vs. `556–870 kg/m²`) and lowers hover power consumption by **>60%** (`~3.5–4.5+ g/W` vs. `~1.2–2.2 g/W`). Geometric clearance, gyroscopic precession (`0.89–3.77 N·m` equivalent `0.087–0.37 N·m` at `3–10 rad/s`), and structural finite-element (FEM) audits further identify the exact mechanical and control requirements—specifically `±25°` prop-arm clearance, `≥3.5 kg·cm` dual-axis actuation, and `3S` powertrain standardization—needed to advance from simulation to flight testing.

---

## I. Introduction

Conventional quadrotors are underactuated systems with four parallel thrust vectors, requiring the airframe to pitch or roll in order to accelerate horizontally. Tilt-rotor quadcopters overcome this limitation by articulating individual motor pods via servo-driven gimbals, enabling independent 6-DOF force and torque generation, level-body station-keeping in crosswinds, and enhanced fault tolerance [1, 2, 4, 5].

The **Black Panther** platform was originally conceived as a stealth-faceted quad-EDF VTOL airframe (`POD_POSITIONS = (±195.0, ±195.0, 15.0) mm`, `390.0 mm` adjacent pod spacing) housing four `50 mm` 11-blade EDF units inside three concentric gimbal rings (`Outer_Gimbal_Ring`, `Inner_Gimbal_Ring`, and `Innermost_Gimbal_Ring`). However, small-diameter EDFs suffer from high disk loading, high current draw (`~32.9–40 A` per pod on `4S`), and poor static hover efficiency (`~1.2–2.2 g/W`), severely limiting hover endurance. To make the airframe viable for sustained VTOL flight using readily available `A2212/10T 1400KV` outrunners (`27.5 mm` outer diameter), the propulsion system was redesigned around `8045` (`8 × 4.5 in`, `203.2 mm`) open propellers mounted via a custom 4-spoke `A2212_Cradle` inside the existing gimbal envelope.

This paper documents:
1. A quantitative comparison between the `50 mm` EDF and `A2212/10T + 8045` configurations across geometry, mass, power, and hover efficiency;
2. Preliminary 3D OpenFOAM-12 RANS actuator-disk flow visualizations for both propulsion pods; and
3. A critical mechanical, gyroscopic, and structural audit establishing the engineering roadmap to a flight-ready prototype.

---

## II. Related Work & Literature Context

Tilt-rotor and over-actuated multirotor UAVs fall into three primary actuation classes, as categorized by Rashad et al. [16]:

1. **Fixed-Tilt Multirotors (0 Servos):** Rajappa et al. [12] demonstrated 6-DOF pose tracking using six statically canted unidirectional rotors, while Brescianini and D'Andrea [11] achieved 360° omnidirectional flight with eight fixed canted reversible propellers. These avoid servo weight and backlash at the cost of continuous internal thrust cancellation in hover.
2. **Single-Axis Tilt Quadrotors (4 Servos, 8 Actuators):** Ryll et al. [5] proved that four rotors paired with four single-axis arm-tilt servos (`1.32 kg` AUW, `Robbe S3150` `0.37 N·m` servos) achieve full 6-DOF pose tracking. Oosedo et al. [6] extended single-axis tilting to `0°–90°` pitch transitions and flipping maneuvers, and Nemati et al. [7] demonstrated decoupled translational/attitude control in prototype tests. Morphing variants such as `FAST-Hex` [13] and `OmniMorph` [14] further reduce actuator count to a single central servo.
3. **Dual-Axis Tilt Quadrotors (8 Servos, 12 Actuators):** Bin Junaid et al. [1] implemented a 12-actuator dual-axis quadcopter using a `1:3` geared arm-rotation drive and push-pull linkage, demonstrating 6-DOF tracking and motor-failure fault tolerance outdoors. Zheng et al. (*TiltDrone*) [4] used twin perpendicular linear servos with explicit geometric constraint optimization (`D_prop` vs. arm clearance) to hover at `30°` body tilt without prop-arm collisions. Ahmed and Katupitiya [2] derived the coupled nonlinear multi-body dynamics and rotor gyroscopic precession terms for 12-input gimbaled quadrotors, and Maadani et al. [3] developed redundant control allocation under aerodynamic wake interactions. For omnidirectional hexarotors, Kamel et al. (*Voliro*) [8], Bodie et al. [9], and Allenspach et al. [10] established motion-capture flight and physical contact-inspection benchmarks, while Moaad et al. [15] analyzed ducted-fan/EDF VTOL sizing.

Relative to this literature, **Black Panther** sits in the 12-actuator dual-axis quadrotor family [1, 2, 3, 4], focusing specifically on the practical retrofit trade-offs when converting a compact ducted-EDF gimbal airframe to high-efficiency `A2212/10T + 8045` open rotors.

---

## III. Airframe & Propulsion Redesign

### A. Geometric & Mass Comparison
Both architectures share the faceted `Panther_X_Chassis`, `Panther_Battery_Bay`, `Panther_Electronics_Bay`, `Panther_Top_Cover`, `Panther_Landing_Skids`, and four swept arms (`Panther_Arm_FL/FR/RL/RR`) modeled in FreeCAD 1.1.4 (`build_black_panther_quad.py`, `build_gimbal_edf.py`, `build_a2212_prop_drone.py`).

| Parameter | 50 mm 11-Blade EDF (`black_panther_quad_edf`) | A2212/10T 1400KV + 8045 (`black_panther_a2212_quad`) |
|---|---|---|
| **Single-Pod CAD Volume** *(excl. arm)* | `91.09 cm³` | `66.67 cm³` |
| **Full Quad CAD Volume** | `1,385.18 cm³` | `1,321.43 cm³` (`1,193.77 cm³` structural) |
| **Swept Rotor / Annulus Area per Pod** | `17.09 cm²` (`R=25, R_hub=9 mm`); `10.75 cm²` exit | **`322.27 cm²`** (`R=101.6, R_hub=7.2 mm`) — **18.9× larger** |
| **Published Unit Mass** | `~76–77 g` (FMS V2 / Powerfun 50 mm) | `~47–55 g` bare motor + `~7–9 g` prop |
| **Published Static Thrust / Pod** | `850–1,086 gf` (`4S`, `32.9–40 A`, `~458–486 W`) | `550–710 gf` sustained (`~750–900 gf` peak, `3S`, `10–14 A`) |
| **Disk Loading** | `~556–870 kg/m²` (nozzle exit) | **`~21.7 kg/m²`** (**25–40× lower**) |
| **Hover Efficiency** | `~1.2–2.2 g/W` | **`~3.5–4.5+ g/W`** (`~5–7 g/W` mid-throttle) |
| **Recommended Battery** | `4S` LiPo | **`2S–3S` LiPo (`7.4–11.1 V`) ONLY** |

### B. Electrical & Structural Compatibility Findings
1. **Voltage Standardization (`3S` vs. `4S`/`6S`):** Published specifications for the `A2212/10T 1400KV` limit continuous/burst current to `12–16 A` (`60 s`) on `2S–3S` LiPo (`~150–180 W` max). Earlier EDF design files referenced `4S` (`BOM_parts_list.md`) and `6S` (`run_cfd_and_ai_audit.py`), which would thermally overload an `A2212/10T` swinging an `8045` prop. Standardizing on **3S (`11.1 V`)** resolves this risk.
2. **All-Up Weight (AUW) & Thrust-to-Weight Ratio (TWR):** With `1,193.77 cm³` of structural CAD volume, a 30%-infill printed airframe (`~600 g`) plus `3S` electronics (`~788 g`) yields `AUW ≈ 1.39–1.55 kg`. Against `4 × 550–710 gf` (`2.20–2.84 kgf`) less `~8–14%` ring/arm blockage (`~1.95–2.55 kgf` effective), effective TWR is **`1.35–1.76 : 1`**, motivating hollowing of `Panther_Top_Cover` (`174.06 cm³`) and `Panther_Battery_Bay` (`369.25 cm³`) to bring AUW below `1.30 kg` (`TWR ≥ 1.85 : 1`).
3. **Tilt-Clearance & Gimbal Kinematics:** Adjacent `8045` props have `186.8 mm` tip-to-tip clearance at neutral (`0°`). However, because the `8045` hub sits at `Z_world = 48.13–59.07 mm` above the repurposed EDF arm (`Z_top = 22.5 mm`), inward tangential tilt causes swept-disk collision with `Panther_Arm_FL` starting at **`15°`** (`78.0 mm³` overlap; `2,124.3 mm³` at `18°`) and with `Servo_Yaw_SG90` at **`≥20°`**. Furthermore, only the outer gimbal axis is actuated (`4` servos) in the current CAD; adding the second axis actuator and raising the motor mount by `25–35 mm` (following Zheng et al. [4]) is required for `±25°` collision-free vectoring.
4. **Gyroscopic Precession Torque:** For the `A2212` bell + `8045` prop ($I_{\text{rotor}} \approx 2.5\text{–}3.2 \times 10^{-5}\text{ kg}\cdot\text{m}^2$) at `11,000 RPM` ($\omega = 1,152\text{ rad/s}$), rapid gimbal slewing ($\Omega_{\text{tilt}} = 3\text{–}10\text{ rad/s}$) produces orthogonal gyroscopic precession torque $\tau_{\text{gyro}} = I\omega\Omega_{\text{tilt}} \approx 0.087\text{–}0.37\text{ N}\cdot\text{m}$ (`0.89–3.77 kg·cm`), exceeding micro-servo ratings (`1.6–2.2 kg·cm`) and requiring `≥3.5–5.0 kg·cm` digital metal-gear servos or geared linkages [1, 5].

---

## IV. Aerodynamic CFD Analysis (OpenFOAM-12)

### A. Numerical Setup
Single-pod steady RANS simulations (`foamRun`, `incompressibleFluid` SIMPLE solver, $k$-$\omega$ SST turbulence model, `bounded Gauss upwind`, wall functions) were executed in OpenFOAM-12 (`blueCFD-Core-2024`) on `snappyHexMesh` grids:
- **50 mm Bellmouth EDF Pod (`run_openfoam_3d_cfd.py`):** `65,952` cells in a `0.15 × 0.15 × 0.21 m` domain, `inlet U = (0, 0, -10) m/s`, `fanDisk` (`3,043` cells, `semiImplicitSource` `explicit (0 0 -220000) m/s²`, `F_applied = 6.65 N`).
- **A2212/10T + 8045 Pod (`run_openfoam_a2212_cfd.py`):** `22,342` cells in a `0.44 × 0.44 × 0.46 m` domain, `inlet U = (0, 0, -3) m/s`, `prop8045Disk` (`204` cells, `explicit (0 0 -17000) m/s²`, `F_applied = 10.34 N`).

### B. Flow-Field Results & Modeling Limitations
1. **50 mm Bellmouth EDF:** The two-stage bellmouth + `2.8 mm` rolled torus lip eliminates outer-lip flow separation, generating a peak jet velocity of **`71.13 m/s`** (`59.70 m/s` mean nozzle exit, `0.0786 kg/s` mass flow, `p ∈ [-1,641, +3,305] Pa`, `3.91 N` / `399 gf` momentum-box net thrust).
2. **A2212 + 8045 Open Rotor:** The `203.2 mm` actuator disk produces a clean contracting slipstream (`23.31 m/s` peak downwash, `16.03 m/s` mean wake at `Z = -80 mm`, `0.557 kg/s` mass flow, `p ∈ [-250, +239] Pa`, `7.25 N` / `739 gf` reported wake thrust, `2,956 gf` quad total).

**Figure References (Workspace Deliverables):**
- 50 mm Bellmouth EDF Contours & Streamlines: [openfoam_3d_cfd.png](file:///c:/Users/shaik/Desktop/new%20drone%20desigen/fem_results/openfoam_3d_cfd.png), [openfoam_streamlines_turbulence.png](file:///c:/Users/shaik/Desktop/new%20drone%20desigen/fem_results/openfoam_streamlines_turbulence.png)
- A2212/10T + 8045 Contours & Streamlines: [openfoam_a2212_8045_cfd.png](file:///c:/Users/shaik/Desktop/new%20drone%20desigen/fem_results/openfoam_a2212_8045_cfd.png)

**CFD Limitations Noted for Future Refinement:** Both preliminary runs prescribe axial-only body forces without blade swirl ($U_\theta = 0$), omit prism boundary layers (`addLayers false`), exclude `Outer_Gimbal_Ring` + arm blockage, and—in the A2212 case—exhibit `16.7%` side-wall confinement (`+1.46 m/s` upward recirculation). Future validation requires a `≥10D` open domain, Rotating/Multiple Reference Frame (MRF) or sliding-mesh (AMI) blade resolution, tilted-pod (`0°–25°`) cases, and a 3-mesh Grid Convergence Index (GCI) study.

---

## V. Structural FEM Audit & Remediation

Preliminary CalculiX (`ccx.EXE`) linear-elastic tests (`run_fem_tests.py`) on the baseline frame revealed two critical solver/modeling issues that must be corrected before citing structural safety factors:
1. Assigning `force.DirectionVector` in FreeCAD without a valid `Part::Line` edge in `force.Direction` caused `Panther_Arm_FL` and `Gimbal_Drive_Shaft` to solve under axial tension (`1.077 MPa` reported on the shaft vs. `~41.6 MPa` analytical cantilever bending, `SF ≈ 5.2`).
2. `Panther_X_Chassis` (`5` solids) and `Panther_Arm_FL` (`2` solids) contained disconnected sub-solids (`0.7 mm` Z-gap on chassis tabs; detached `rear_pillow`), and `Prop_8045` (`3` solids) had a `2.8 mm` hub-blade gap.

Correcting these solid booleans, applying pin-bore bearing loads at `Ø3 mm` pivots, and running combined thrust + centrifugal + gyroscopic FEM on `black_panther_a2212_quad.FCStd` are immediate pre-fabrication steps.

---

## VI. Conclusion

Converting the Black Panther quadrotor from four `50 mm` EDFs to four `A2212/10T 1400KV` motors with `8045` propellers on `3S` LiPo power increases swept rotor area by **18.9×** and reduces hover power by **>60%**, making sustained VTOL flight feasible. Realizing a reliable 2-axis thrust-vectoring prototype requires: (1) adding the second gimbal servo/linkage, (2) raising the motor mount `25–35 mm` for `±25°` prop-arm clearance, (3) upgrading to `≥3.5–5.0 kg·cm` digital metal-gear servos to counter `0.89–3.77 kg·cm` gyroscopic precession torque, and (4) implementing a 12-to-6-DOF control allocator.

---

## VII. Pre-Submission Validation Checklist `[TODO Before Finalizing Paper]`

- [ ] **Fix CAD & Re-run FEM:** Fuse `Prop_8045`, `Panther_X_Chassis`, and `Panther_Arm_FL`; add 2nd gimbal servo; raise motor mount `≥25 mm`; fix `Force.Direction` in `run_fem_tests.py` and insert updated `MaxVM` / `SF` values in Section V.
- [ ] **Bench Thrust-Stand Test (`3S` LiPo):** Measure static thrust (`gf`), current (`A`), voltage (`V`), RPM, and `g/W` vs. throttle (`20%–100%`) for `A2212/10T 1400KV + 8045` (both unobstructed and mounted over the gimbal ring to quantify ring blockage).
- [ ] **Bench Servo Torque / Deflection Check:** Command `±20°` gimbal sweeps at `50–100%` throttle to verify zero servo stalling or gear slip under gyroscopic load.
- [ ] *(For IEEE/AIAA Conference or Journal):* Add 6-DOF Simulink/SITL trajectory tracking plots, 3-mesh GCI + MRF CFD results, and indoor/outdoor flight telemetry.

---

## References

1. A. Bin Junaid, A. D. Sanchez, J. B. Bosch, N. Vitzilaios, and Y. Zweiri, "Design and Implementation of a Dual-Axis Tilting Quadcopter," *MDPI Robotics*, vol. 7, no. 4, p. 65, 2018. DOI: `10.3390/robotics7040065`
2. N. Ahmed and J. Katupitiya, "Modeling and Control of a Novel Vectored-Thrust Quadcopter," *AIAA Journal of Guidance, Control, and Dynamics*, vol. 44, no. 7, pp. 1399–1409, 2021. DOI: `10.2514/1.G005467`
3. M. Maadani, P. López Muro, E. Soriano, and L. Burlion, "Control Allocation of Over-Actuated Tilt-Rotor Quadcopters with Aerodynamic Interactions," *AIAA SciTech 2023 Forum*, 2023. DOI: `10.2514/6.2023-2243`
4. P. Zheng, X. Tan, B. B. Kocer, E. Yang, and M. Kovac, "TiltDrone: A Fully-Actuated Tilting Quadrotor Platform," *IEEE Robotics and Automation Letters*, vol. 5, no. 4, pp. 6845–6852, 2020. DOI: `10.1109/LRA.2020.3010460`
5. M. Ryll, H. H. Bülthoff, and P. R. Giordano, "A Novel Overactuated Quadrotor Unmanned Aerial Vehicle: Modeling, Control, and Experimental Validation," *IEEE Transactions on Control Systems Technology*, vol. 23, no. 2, pp. 540–556, 2015. DOI: `10.1109/TCST.2014.2330999`
6. A. Oosedo, S. Abiko, S. Narasaki, A. Kuno, A. Konno, and M. Uchiyama, "Large attitude change flight of a quad tilt rotor unmanned aerial vehicle," *Advanced Robotics*, vol. 30, no. 5, pp. 326–337, 2016. DOI: `10.1080/01691864.2015.1134344`
7. A. Nemati, N. Soni, M. Sarim, and M. Kumar, "Design, Fabrication and Control of a Tilt Rotor Quadcopter," *ASME Dynamic Systems and Control Conference (DSCC)*, vol. 2, 2016. DOI: `10.1115/DSCC2016-9929`
8. M. Kamel, S. Verling, O. Elkhatib, C. Sprecher, P. Wulkop, Z. Taylor, R. Siegwart, and I. Gilitschenski, "The Voliro Omniorientational Hexacopter: An Agile and Maneuverable Tiltable-Rotor Aerial Vehicle," *IEEE Robotics & Automation Magazine*, vol. 25, no. 4, pp. 34–44, 2018. DOI: `10.1109/MRA.2018.2866758`
9. K. Bodie, M. Brunner, M. Pantic, et al., "Active Interaction Force Control for Contact-Based Inspection With a Fully Actuated Aerial Vehicle," *IEEE Transactions on Robotics*, 2020. DOI: `10.1109/TRO.2020.3036623`
10. M. Allenspach, K. Bodie, M. Brunner, L. Tognon, and R. Siegwart, "Design and optimal control of a tiltrotor micro-aerial vehicle for efficient omnidirectional flight," *International Journal of Robotics Research*, 2020. DOI: `10.1177/0278364920943654`
11. D. Brescianini and R. D'Andrea, "An omni-directional multirotor vehicle," *Elsevier Mechatronics*, vol. 55, pp. 76–93, 2018. DOI: `10.1016/j.mechatronics.2018.08.005`
12. S. Rajappa, M. Ryll, H. H. Bülthoff, and A. Franchi, "Modeling, control and design optimization for a fully-actuated hexarotor aerial vehicle with tilted propellers," *IEEE International Conference on Robotics and Automation (ICRA)*, pp. 4006–4013, 2015. DOI: `10.1109/ICRA.2015.7139759`
13. M. Ryll, D. Bicego, M. Giurato, M. Lovera, and A. Franchi, "FAST-Hex — A Morphing Hexarotor: Design, Mechanical Implementation, Control and Experimental Validation," *IEEE/ASME Transactions on Mechatronics*, 2022. DOI: `10.1109/TMECH.2021.3099197`
14. Y. Aboudorra, C. Gabellieri, R. Brantjes, Q. Sablé, and A. Franchi, "Modelling, Analysis and Control of OmniMorph: An Omnidirectional Morphing Multi-rotor UAV," *Journal of Intelligent & Robotic Systems*, vol. 110, p. 21, 2024. DOI: `10.1007/s10846-024-02054-x`
15. A. Moaad et al., "Design and optimization of a ducted fan VTOL MAV controlled by Electric Ducted Fans," *EUCASS*, 2019. DOI: `10.13009/EUCASS2019-108`
16. R. Rashad, J. Goerres, R. Aarts, J. B. C. Engelen, and S. Stramigioli, "Fully Actuated Multirotor UAVs: A Literature Review," *IEEE Robotics & Automation Magazine*, vol. 27, no. 3, pp. 97–107, 2020. DOI: `10.1109/MRA.2019.2955964`
