#!/usr/bin/env python3
"""Prepare one map-frame submap for both localization replays."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np

from drone_mapping import mapio


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("reloc_config", type=Path)
    crop = parser.add_mutually_exclusive_group()
    crop.add_argument("--xy", type=float, nargs=4, metavar=("MIN_X", "MIN_Y", "MAX_X", "MAX_Y"))
    crop.add_argument("--height-only", action="store_true",
                      help="keep all points in the height band, without XY clipping or downsampling")
    crop.add_argument("--xy-polygon", type=Path,
                      help="clip to a convex polygon in drone_map XY, preserving point density")
    parser.add_argument("--grid-memory-mib", type=int, default=512)
    args = parser.parse_args()
    source, output, reloc = (p.resolve() for p in (args.source, args.output, args.reloc_config))
    meta = json.loads((source / "meta.json").read_text())
    floor = meta["floor"]
    if not floor.get("trusted") or not math.isfinite(float(floor["height_m"])):
        raise ValueError("source map needs a validated floor reference")
    ground = -float(floor["height_m"])
    xmin, ymin, xmax, ymax = args.xy or (-24.0, -28.0, 32.0, 28.0)
    lower = np.array([xmin, ymin, ground - 0.5])
    upper = np.array([xmax, ymax, ground + 6.0])
    if not np.isfinite([lower, upper]).all() or np.any(lower >= upper):
        raise ValueError("invalid crop bounds")
    points, intensity = mapio.read_pcd(str(source / "map_world.pcd"))
    polygon = None
    if args.xy_polygon:
        polygon = np.asarray(json.loads(args.xy_polygon.read_text())["xy_polygon_m"], dtype=float)
        if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2 or not np.isfinite(polygon).all():
            raise ValueError("expected at least three finite XY polygon vertices")
        sides = np.roll(polygon, -1, axis=0) - polygon
        turns = np.cross(sides, np.roll(sides, -1, axis=0))
        if not (np.all(turns > 0) or np.all(turns < 0)):
            raise ValueError("polygon vertices must form a strictly convex boundary")
        selected = (points[:, 2] >= lower[2]) & (points[:, 2] <= upper[2])
        turn = np.sign(turns[0])
        for vertex, side in zip(polygon, sides):
            selected &= turn * np.cross(side, points[:, :2] - vertex) >= 0
        requested_min = [*polygon.min(axis=0), lower[2]]
        requested_max = [*polygon.max(axis=0), upper[2]]
    elif args.height_only:
        selected = (points[:, 2] >= lower[2]) & (points[:, 2] <= upper[2])
        requested_min, requested_max = [None, None, lower[2]], [None, None, upper[2]]
    else:
        selected = ((points >= lower) & (points <= upper)).all(axis=1)
        requested_min, requested_max = lower.tolist(), upper.tolist()
    if selected.sum() < 1000:
        raise ValueError("crop contains too few map points")
    xyz, intensity = points[selected], intensity[selected]
    if not args.height_only and polygon is None:
        xyz, intensity = mapio.voxel_down(xyz, intensity, 0.10)
    lo, hi = xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()
    # Same padding and float storage as the default DLL distance field.
    dims = [math.ceil((b + 1.1 - (a - 1.1)) / 0.10) + 1 for a, b in zip(lo, hi)]
    size = math.prod(dims) * 4
    if args.grid_memory_mib <= 0 or size > args.grid_memory_mib * 1024**2:
        raise ValueError(f"submap needs {size / 1024**2:.1f} MiB, DLL budget is {args.grid_memory_mib} MiB")
    output.mkdir(parents=True)
    mapio.write_pcd(str(output / "map_world.pcd"), xyz, intensity)
    shutil.copyfile(reloc, output / "outdoor_reloc.yaml")
    origin = dict(map_dir=str(source), map_sha256=digest(source / "map_world.pcd"),
                  meta_sha256=digest(source / "meta.json"))
    if not args.height_only and polygon is None:
        meta["voxel_m"] = 0.10
    meta.update(points_world=len(xyz), extent_world=dict(min=lo, max=hi),
                shared_outdoor_crop=dict(source=origin, frame_id="drone_map",
                                         requested_min=requested_min, requested_max=requested_max,
                                         xy_polygon_m=polygon.tolist() if polygon is not None else None))
    (output / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n")
    report = dict(source=origin, map_dir=str(output), map_frame="drone_map",
                  requested_min=requested_min, requested_max=requested_max, points=len(xyz),
                  xy_polygon_m=polygon.tolist() if polygon is not None else None,
                  points_before_downsample=int(selected.sum()), bounds=meta["extent_world"],
                  dll_field_dimensions=dims, dll_field_mib=size / 1024**2,
                  dll_resolution_m=0.10, dll_grid_memory_mib=args.grid_memory_mib,
                  sha256={name: digest(output / name)
                          for name in ("map_world.pcd", "meta.json", "outdoor_reloc.yaml")})
    encoded = json.dumps(report, indent=2)
    (output / "manifest.json").write_text(encoded + "\n")
    print(encoded)


if __name__ == "__main__":
    main()
