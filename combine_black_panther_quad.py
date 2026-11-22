"""Combine the Black Panther frame and all 4 gimbaled EDF pods into one unified model."""

import os
import FreeCAD as App
import MeshPart
import Part


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(script_dir, "black_panther_quad_edf.FCStd")
    out_fcstd = os.path.join(script_dir, "black_panther_quad_edf_combined.FCStd")
    out_step = os.path.join(script_dir, "black_panther_quad_edf_combined.step")
    out_stl = os.path.join(script_dir, "black_panther_quad_edf_combined.stl")

    src_doc = App.openDocument(src_path)
    shapes = [
        obj.Shape
        for obj in src_doc.Objects
        if hasattr(obj, "Shape") and obj.Shape is not None and not obj.Shape.isNull()
    ]
    App.closeDocument(src_doc.Name)

    combined_shape = Part.makeCompound(shapes)

    out_doc = App.newDocument("BlackPantherQuadEDF_Combined")
    obj = out_doc.addObject("Part::Feature", "BlackPanther_Quad_EDF_Combined")
    obj.Shape = combined_shape
    out_doc.recompute()

    out_doc.saveAs(out_fcstd)
    Part.export([obj], out_step)

    mesh = MeshPart.meshFromShape(Shape=combined_shape, LinearDeflection=0.2, AngularDeflection=0.35)
    mesh.write(out_stl)

    print(f"Combined {len(shapes)} parts into:")
    print(f"  FCStd: {out_fcstd}")
    print(f"  STEP : {out_step}")
    print(f"  STL  : {out_stl}")


if __name__ in ("__main__", "combine_black_panther_quad"):
    main()
