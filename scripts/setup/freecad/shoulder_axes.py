import FreeCAD
doc = FreeCAD.ActiveDocument
rows = set()

for pname in ["ST3215_v1__1_", "ST3215_v1__2_"]:
    part = doc.getObject(pname)
    for o in part.OutListRecursive:
        if o.isDerivedFrom("Part::Feature") and not o.Shape.isNull():
            shp = o.Shape.copy()
            try:
                shp.Placement = o.getGlobalPlacement()
            except Exception:
                pass
            for f in shp.Faces:
                s = f.Surface
                if s.__class__.__name__ == "Cylinder" and 3 < s.Radius < 8:
                    c = s.Center
                    a = s.Axis
                    rows.add((pname, round(s.Radius, 1),
                              round(c.x, 1), round(c.y, 1), round(c.z, 1),
                              round(a.x, 2), round(a.y, 2), round(a.z, 2)))

for r in sorted(rows):
    print(r)
print("rows:", len(rows))
