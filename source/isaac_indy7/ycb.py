"""YCB object spawning helpers."""

from __future__ import annotations

import math
import random

import numpy as np


def yaw_to_quat(yaw: float) -> np.ndarray:
    """Convert yaw around world Z to an Isaac wxyz quaternion."""
    half = yaw * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def spawn_ycb(cfg: dict, base_position=(0.0, 0.0, 0.0)) -> list[str]:
    """Spawn YCB objects from an inline config dictionary."""
    from isaacsim.core.prims import SingleXFormPrim
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("get_assets_root_path() returned None; check Nucleus/assets access")

    usd_dir = cfg["usd_dir"]
    objects = cfg["objects"]
    count = int(cfg.get("count", len(objects)))
    spawn = cfg["spawn"]
    scale = float(spawn.get("scale", 1.0))

    rng = random.Random(cfg.get("seed", None))
    base = np.asarray(base_position, dtype=float)
    spawned: list[str] = []

    for i in range(count):
        usd_name = rng.choice(objects)
        usd_path = assets_root + usd_dir + usd_name
        prim_path = f"/World/ycb/obj_{i:02d}"

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        radius = rng.uniform(spawn["radius_min"], spawn["radius_max"])
        theta = rng.uniform(spawn["angle_min"], spawn["angle_max"])
        position = np.array(
            [
                base[0] + radius * math.cos(theta),
                base[1] + radius * math.sin(theta),
                base[2] + float(spawn["z"]),
            ],
            dtype=float,
        )
        yaw = rng.uniform(-math.pi, math.pi)

        SingleXFormPrim(
            prim_path=prim_path,
            position=position,
            orientation=yaw_to_quat(yaw),
            scale=np.array([scale, scale, scale]),
        )
        spawned.append(prim_path)
        print(f"[ycb] spawned {usd_name} -> {prim_path} @ {position.round(3).tolist()}")

    return spawned


def print_ycb_centers(ycb_paths: list[str]) -> None:
    from isaacsim.core.prims import SingleXFormPrim

    for path in ycb_paths:
        pos, quat = SingleXFormPrim(path).get_world_pose()
        print(
            f"[ycb] center {path}: "
            f"pos={np.asarray(pos).round(4).tolist()}, "
            f"quat={np.asarray(quat).round(4).tolist()}"
        )
