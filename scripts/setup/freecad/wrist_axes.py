import FreeCAD
doc = FreeCAD.ActiveDocument
part = doc.getObject("ST3215_v1__4_")
groups = {}

for o in part.OutListRecursive:
    if o.isDerivedFrom("Part::Feature") and not o.Shape.isNull():
        shp = o.Shape.copy()
        try:
            shp.Placement = o.getGlobalPlacement()
        except Exception:
            pass
        for f in shp.Faces:
            s = f.Surface
            if s.__class__.__name__ == "Cylinder" and 2 < s.Radius < 10:
                a = s.Axis
                c = s.Center
                if (abs(a.x), abs(a.y), abs(a.z)) < (0.5, 0.5, 0.5) and False:
                    continue
                # make axis direction sign-independent
                if a.x < -1e-6 or (abs(a.x) < 1e-6 and a.y < -1e-6) or (abs(a.x) < 1e-6 and abs(a.y) < 1e-6 and a.z < 0):
                    a = FreeCAD.Vector(-a.x, -a.y, -a.z)
                d = c.x * a.x + c.y * a.y + c.z * a.z
                p = FreeCAD.Vector(c.x - a.x * d, c.y - a.y * d, c.z - a.z * d)
                key = (round(a.x, 2), round(a.y, 2), round(a.z, 2),
                       round(p.x, 1), round(p.y, 1), round(p.z, 1))
                groups.setdefault(key, set()).add(round(s.Radius, 1))

print("== wrist servo, axes with 4 or more different radii")
for k in sorted(groups):
    if len(groups[k]) >= 4:
        print("axis=(%s,%s,%s) point_on_axis=(%s,%s,%s) radii=%s" % (k + (sorted(groups[k]),)))
print("groups:", len(groups))
