import FreeCAD
doc = FreeCAD.ActiveDocument

for pname in ["ST3215_v1__3_", "ST3215_v1__4_"]:
    part = doc.getObject(pname)
    found = {}
    for o in part.OutListRecursive:
        if o.isDerivedFrom("Part::Feature") and not o.Shape.isNull():
            shp = o.Shape.copy()
            try:
                shp.Placement = o.getGlobalPlacement()
            except Exception:
                pass
            for f in shp.Faces:
                s = f.Surface
                if s.__class__.__name__ == "Cylinder" and 2 < s.Radius < 9 and abs(s.Axis.y) > 0.95:
                    key = (round(s.Center.x, 1), round(s.Center.z, 1))
                    found.setdefault(key, set()).add(round(s.Radius, 1))
    print("==", pname)
    for k in sorted(found):
        print("x=%s z=%s radii=%s" % (k[0], k[1], sorted(found[k])))
