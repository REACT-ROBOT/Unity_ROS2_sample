#!/usr/bin/env python3

import argparse
import math


def naca_00xx_profile(chord, thickness_ratio, points):
    profile = []
    for i in range(points):
        x = chord * (i / (points - 1))
        xc = x / chord
        yt = 5.0 * thickness_ratio * chord * (
            0.2969 * math.sqrt(xc)
            - 0.1260 * xc
            - 0.3516 * xc * xc
            + 0.2843 * xc * xc * xc
            - 0.1015 * xc * xc * xc * xc
        )
        profile.append((x, yt))
    upper = profile
    lower = [(x, -z) for (x, z) in reversed(profile[1:-1])]
    return upper + lower


def triangle_normal(p1, p2, p3):
    ux, uy, uz = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
    vx, vy, vz = (p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2])
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length == 0:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def write_ascii_stl(path, triangles, solid_name):
    with open(path, "w", encoding="ascii") as f:
        f.write(f"solid {solid_name}\n")
        for (p1, p2, p3) in triangles:
            nx, ny, nz = triangle_normal(p1, p2, p3)
            f.write(f"  facet normal {nx:.6f} {ny:.6f} {nz:.6f}\n")
            f.write("    outer loop\n")
            f.write(f"      vertex {p1[0]:.6f} {p1[1]:.6f} {p1[2]:.6f}\n")
            f.write(f"      vertex {p2[0]:.6f} {p2[1]:.6f} {p2[2]:.6f}\n")
            f.write(f"      vertex {p3[0]:.6f} {p3[1]:.6f} {p3[2]:.6f}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {solid_name}\n")


def build_wing_mesh(profile, span):
    triangles = []
    y0 = -span * 0.5
    y1 = span * 0.5

    # Side surfaces
    for i in range(len(profile)):
        p0 = profile[i]
        p1 = profile[(i + 1) % len(profile)]

        v0 = (p0[0], y0, p0[1])
        v1 = (p1[0], y0, p1[1])
        v2 = (p1[0], y1, p1[1])
        v3 = (p0[0], y1, p0[1])

        triangles.append((v0, v1, v2))
        triangles.append((v0, v2, v3))

    # Caps
    cx = sum(p[0] for p in profile) / len(profile)
    cz = sum(p[1] for p in profile) / len(profile)

    center_root = (cx, y0, cz)
    center_tip = (cx, y1, cz)

    for i in range(len(profile)):
        p0 = profile[i]
        p1 = profile[(i + 1) % len(profile)]

        v0 = (p0[0], y0, p0[1])
        v1 = (p1[0], y0, p1[1])
        triangles.append((center_root, v1, v0))

        v2 = (p0[0], y1, p0[1])
        v3 = (p1[0], y1, p1[1])
        triangles.append((center_tip, v2, v3))

    return triangles


def main():
    parser = argparse.ArgumentParser(description="Generate a simple NACA 00xx wing STL.")
    parser.add_argument("--chord", type=float, default=0.4)
    parser.add_argument("--span", type=float, default=1.2)
    parser.add_argument("--thickness", type=float, default=0.12)
    parser.add_argument("--points", type=int, default=50)
    parser.add_argument("--output", required=True)
    parser.add_argument("--name", default="wing")
    args = parser.parse_args()

    profile = naca_00xx_profile(args.chord, args.thickness, args.points)
    triangles = build_wing_mesh(profile, args.span)
    write_ascii_stl(args.output, triangles, args.name)


if __name__ == "__main__":
    main()
