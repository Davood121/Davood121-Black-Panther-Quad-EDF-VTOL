"""Export an exploded STEP assembly of black_panther_quad_edf.FCStd for Onshape."""

import os
import FreeCAD as App
import Part

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
doc = App.openDocument(os.path.join(BASE_DIR, "black_panther_quad_edf.FCStd"))

center_z = {
    "Panther_Landing_Skids": -75.0,
    "Panther_Battery_Bay": -40.0,
    "Panther_X_Chassis": 0.0,
    "Panther_Electronics_Bay": 40.0,
    "Panther_Top_Edge_Lights": 68.0,
    "Panther_Top_Cover": 95.0,
}
for name, dz in center_z.items():
    o = doc.getObject(name)
    if o:
        o.Placement.Base = o.Placement.Base.add(App.Vector(0, 0, dz))

pods = {
    "FL_": App.Vector(195, 195, 15),
    "FR_": App.Vector(195, -195, 15),
    "RL_": App.Vector(-195, 195, 15),
    "RR_": App.Vector(-195, -195, 15),
}
for prefix, c in pods.items():
    inward = App.Vector(-c.x, -c.y, 0)
    inward.normalize()
    outward = inward.multiply(-1.0)
    shifts = {
        f"Panther_Arm_{prefix.rstrip('_')}": outward.multiply(55.0),
        f"{prefix}Servo_Yaw_SG90": outward.multiply(55.0).add(App.Vector(0, 0, -28.0)),
        f"{prefix}Spur_Gear_Lower": outward.multiply(55.0).add(App.Vector(0, 0, 24.0)),
        f"{prefix}Spur_Gear_Upper": outward.multiply(28.0).add(App.Vector(0, 0, 24.0)),
        f"{prefix}Gimbal_Drive_Shaft": outward.multiply(18.0),
        f"{prefix}Outer_Pivot_Pins": outward.multiply(-20.0),
        f"{prefix}Inner_Gimbal_Ring": outward.multiply(65.0),
        f"{prefix}Innermost_Gimbal_Ring": outward.multiply(65.0).add(App.Vector(0, 0, 34.0)),
        f"{prefix}Inner_Pivot_Pins": outward.multiply(65.0).add(App.Vector(0, 0, 52.0)),
        f"{prefix}EDF_Mount_Bolts": outward.multiply(65.0).add(App.Vector(0, 0, 70.0)),
        f"{prefix}EDF_Shroud": outward.multiply(65.0).add(App.Vector(0, 0, 98.0)),
        f"{prefix}EDF_Rotor_11Blade": outward.multiply(65.0).add(App.Vector(0, 0, 148.0)),
        f"{prefix}EDF_Exhaust_Nozzle": outward.multiply(65.0).add(App.Vector(0, 0, -58.0)),
    }
    for oname, vec in shifts.items():
        o = doc.getObject(oname)
        if o:
            o.Placement.Base = o.Placement.Base.add(vec)

doc.recompute()
out_path = os.path.join(BASE_DIR, "black_panther_exploded.step")
Part.export(doc.Objects, out_path)
print("EXPORTED_EXPLODED_STEP:", out_path)
