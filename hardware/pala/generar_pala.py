"""Fixed scoop for Phineas/Ferb: parametric 3D model -> STL (print) and OpenSCAD (edit).

Geometry = the scoop validated in Webots (RecolectorUCAB.proto, default fields): U pocket 40 mm wide
inside, 48 mm deep, 3 mm walls starting 2.5 mm above the floor, and a 44 deg funnel (~10 cm mouth).
The cube (30 mm) rests against the back wall; reversing leaves it on the floor.

One flat plate carries everything: its top face screws to the underside of the chassis plate and the
walls hang from it. Printed upside down, the whole plate lies on the bed and the walls grow upward,
so no supports are needed (a separate, higher mounting tongue would have left the plate in the air).

The mounting tongue is the only part that depends on the real chassis: measure it and edit the
MOUNT_* values (see hardware/pala/README.md) before printing.

Run with Python 3 and the manifold3d package:
    python generar_pala.py            # writes pala_fija.stl and pala_fija.scad next to this file

Frame: x forward (0 = front edge of the chassis plate), y to the left, z up (0 = floor), millimeters.
"""

import math
import pathlib
import struct

import manifold3d as mf
import numpy as np

# ------------------------------------------------------------------ parameters (mm)
WALL = 3.0                # wall thickness
FLOOR_GAP = 2.5           # walls start this high above the floor (they must not drag)
POCKET_WIDTH = 40.0       # inside width of the U (30 mm cube + 5 mm per side)
POCKET_DEPTH = 48.0       # inside depth of the U
BACK_WALL_X = 38.0        # x of the back wall center (cube center ends ~145 mm ahead of the robot center)
FUNNEL_LENGTH = 39.6      # each funnel wall
FUNNEL_ANGLE = 44.0       # deg, outward from the robot axis (mouth ~10 cm)
PLATE_FRONT = 42.0        # the plate reaches x = 42 (over the back wall)
PLATE_WIDTH = 50.0

# Mounting (MEASURE ON THE ROBOT, see README.md).
MOUNT_UNDERSIDE_Z = 44.0  # floor to the underside of the chassis plate the scoop screws to
MOUNT_LENGTH = 35.0       # how far the plate goes under the chassis (x from 0 to -MOUNT_LENGTH)
PLATE_THICK = 4.0
SLOT_SPACING = 30.0       # distance between the two M3 slots (y)
SLOT_CENTER_X = -20.0     # slot centers
SLOT_LENGTH = 12.0        # slots along x: +-4.3 mm of front/back adjustment
SLOT_WIDTH = 3.4          # M3 clearance

HERE = pathlib.Path(__file__).resolve().parent


# ------------------------------------------------------------------ parts as (size, center, yaw deg)
def boxes():
    parts = []
    # Walls reach the plate's top face: printed upside down every wall starts on the bed too
    # (walls ending under the plate left 511 mm2 floating 3.5 mm above the bed).
    z_mid = (FLOOR_GAP + MOUNT_UNDERSIDE_Z) / 2
    z_len = MOUNT_UNDERSIDE_Z - FLOOR_GAP
    half_in = POCKET_WIDTH / 2
    y_wall = half_in + WALL / 2
    x_back_in = BACK_WALL_X + WALL / 2                 # inside face of the back wall
    x_mouth = x_back_in + POCKET_DEPTH
    # back wall (as wide as the outside of the U)
    parts.append(((WALL, POCKET_WIDTH + 2 * WALL, z_len), (BACK_WALL_X, 0.0, z_mid), 0.0))
    for s in (1, -1):
        # side walls of the U (overlap the back wall)
        parts.append(((POCKET_DEPTH + WALL, WALL, z_len), (BACK_WALL_X + (POCKET_DEPTH + WALL) / 2, s * y_wall, z_mid), 0.0))
        # funnel walls, starting at the mouth corner and opening outward
        a = math.radians(FUNNEL_ANGLE)
        cx = x_mouth + FUNNEL_LENGTH / 2 * math.cos(a)
        cy = s * (y_wall + FUNNEL_LENGTH / 2 * math.sin(a))
        parts.append(((FUNNEL_LENGTH, WALL, z_len), (cx, cy, z_mid), s * FUNNEL_ANGLE))
    # one flat plate: under the chassis (x < 0, with the slots) and over the back wall (x up to 42)
    length = MOUNT_LENGTH + PLATE_FRONT
    parts.append(((length, PLATE_WIDTH, PLATE_THICK),
                  (PLATE_FRONT - length / 2, 0.0, MOUNT_UNDERSIDE_Z - PLATE_THICK / 2), 0.0))
    return parts, x_mouth, y_wall


def joint_posts(x_mouth, y_wall):
    """Vertical cylinders filling the corner between each side wall and its funnel wall."""
    z0, z1 = FLOOR_GAP, MOUNT_UNDERSIDE_Z
    return [((x_mouth, s * y_wall), WALL / 2, z0, z1) for s in (1, -1)]


def slots():
    """Slots through the tongue: (center x, center y, length along x, width)."""
    return [(SLOT_CENTER_X, s * SLOT_SPACING / 2, SLOT_LENGTH, SLOT_WIDTH) for s in (1, -1)]


# ------------------------------------------------------------------ solid model
def box(size, center, yaw):
    m = mf.Manifold.cube(list(size), center=True)
    if yaw:
        m = m.rotate([0.0, 0.0, yaw])
    return m.translate(list(center))


def slot_solid(cx, cy, length, width, z0, z1):
    r = width / 2
    straight = length - width
    cyl = mf.Manifold.cylinder(z1 - z0, r, r, 48)
    ends = [cyl.translate([cx + s * straight / 2, cy, z0]) for s in (1, -1)]
    middle = mf.Manifold.cube([straight, width, z1 - z0]).translate([cx - straight / 2, cy - r, z0])
    return ends[0] + ends[1] + middle


def build():
    parts, x_mouth, y_wall = boxes()
    solid = mf.Manifold.batch_boolean([box(*p) for p in parts], mf.OpType.Add)
    for (px, py), r, z0, z1 in joint_posts(x_mouth, y_wall):
        solid = solid + mf.Manifold.cylinder(z1 - z0, r, r, 32).translate([px, py, z0])
    z0, z1 = MOUNT_UNDERSIDE_Z - PLATE_THICK - 1.0, MOUNT_UNDERSIDE_Z + 1.0
    for cx, cy, length, width in slots():
        solid = solid - slot_solid(cx, cy, length, width, z0, z1)
    return solid


def to_print_orientation(solid):
    """Upside down: the top plate and the tongue lie on the bed, the walls grow upward (no supports)."""
    flipped = solid.rotate([180.0, 0.0, 0.0])
    (_x0, _y0, z0, _x1, _y1, _z1) = flipped.bounding_box()
    return flipped.translate([0.0, 0.0, -z0])


def write_stl(solid, path: pathlib.Path):
    mesh = solid.to_mesh()
    verts = np.asarray(mesh.vert_properties)[:, :3]
    tris = np.asarray(mesh.tri_verts)
    with path.open("wb") as f:
        f.write(b"Phineas/Ferb fixed scoop - generar_pala.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            v0, v1, v2 = verts[a], verts[b], verts[c]
            n = np.cross(v1 - v0, v2 - v0)
            norm = np.linalg.norm(n)
            n = n / norm if norm else n
            f.write(struct.pack("<12fH", *n, *v0, *v1, *v2, 0))


def write_scad(path: pathlib.Path):
    """Same geometry as OpenSCAD source (editable in the free OpenSCAD app)."""
    parts, x_mouth, y_wall = boxes()
    lines = [
        "// Phineas/Ferb fixed scoop. Generated by generar_pala.py (same parameters); millimeters.",
        "// x forward (0 = front edge of the chassis plate), y left, z up (0 = floor).",
        "// Rotate 180 deg around X to print (the flat plate on the bed, walls upward, no supports).",
        "$fn = 48;",
        "difference() {",
        "  union() {",
    ]
    for (sx, sy, sz), (cx, cy, cz), yaw in parts:
        lines.append(f"    translate([{cx:.3f}, {cy:.3f}, {cz:.3f}]) rotate([0, 0, {yaw:.3f}]) "
                     f"cube([{sx:.3f}, {sy:.3f}, {sz:.3f}], center = true);")
    for (px, py), r, z0, z1 in joint_posts(x_mouth, y_wall):
        lines.append(f"    translate([{px:.3f}, {py:.3f}, {z0:.3f}]) cylinder(h = {z1 - z0:.3f}, r = {r:.3f});")
    lines.append("  }")
    z0, z1 = MOUNT_UNDERSIDE_Z - PLATE_THICK - 1.0, MOUNT_UNDERSIDE_Z + 1.0
    for cx, cy, length, width in slots():
        straight = length - width
        lines.append(f"  translate([{cx:.3f}, {cy:.3f}, {z0:.3f}]) hull() {{ "
                     f"for (d = [-{straight / 2:.3f}, {straight / 2:.3f}]) translate([d, 0, 0]) "
                     f"cylinder(h = {z1 - z0:.3f}, r = {width / 2:.3f}); }}")
    lines.append("}")
    path.write_text("\n".join(lines) + "\n")


def main():
    solid = build()
    assert solid.status() == mf.Error.NoError, solid.status()
    printable = to_print_orientation(solid)
    write_stl(printable, HERE / "pala_fija.stl")
    write_scad(HERE / "pala_fija.scad")
    x0, y0, z0, x1, y1, z1 = printable.bounding_box()
    volume_cm3 = printable.volume() / 1000.0
    print(f"pala_fija.stl: {printable.num_tri()} triangles, genus {printable.genus()}, "
          f"size {x1 - x0:.1f} x {y1 - y0:.1f} x {z1 - z0:.1f} mm, solid volume {volume_cm3:.1f} cm3 "
          f"(~{volume_cm3 * 1.24:.0f} g of PLA at 100 % infill)")


if __name__ == "__main__":
    main()
