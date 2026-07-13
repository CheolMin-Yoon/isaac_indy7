"""Indy7 scene entrypoint.

Examples:
    ~/isaacsim/python.sh indy7.py
    ~/isaacsim/python.sh indy7.py --target-position 0.45 0.0 0.35
    ~/isaacsim/python.sh indy7.py --target-position 0.45 0.0 0.35 --target-orientation 0 0 1 0
    ~/isaacsim/python.sh indy7.py --gripper close
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_DIR = os.path.join(ROOT_DIR, "source")
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from isaacsim import SimulationApp

INDY7_USD = os.path.join(
    ROOT_DIR,
    "source",
    "assets",
    "indy7_v2",
    "indy7_v2_with_2f-140_d455.usd",
)
INDY7_PRIM_PATH = "/World/indy7"
INDY7_POSITION = [0.0, 0.0, 0.0]
YCB_CONFIG = {
    "usd_dir": "/Isaac/Props/YCB/Axis_Aligned_Physics/",
    "objects": [
        "003_cracker_box.usd",
        "004_sugar_box.usd",
        "005_tomato_soup_can.usd",
        "006_mustard_bottle.usd",
    ],
    "count": 5,
    "spawn": {
        "radius_min": 0.35,
        "radius_max": 0.55,
        "angle_min": -1.2,
        "angle_max": 1.2,
        "z": 0.20,
        "scale": 1.0,
    },
    "seed": 0,
}
CAMERA_CONFIG = {
    # d455 mount + rsd455.usd reference are already baked into INDY7_USD at
    # link6/d455 — WristCamera only re-poses the mount and wraps it.
    "prim_name": "d455",
    "asset_root_name": "RSD455",
    "color_camera": "Camera_OmniVision_OV9782_Color",
    "depth_camera": "Camera_Pseudo_Depth",
    "resolution": [640, 480],
    "mount_translation": [0.06031178594972571, 0.002541999360932995, 0.03876863710717515],
    "mount_orientation": [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
    "clipping_range": [0.01, 5.0],
    "capture_depth": True,
    "capture_every": 60,
    "output_dir": "output/camera",
}
PHYSICS_DT = 1.0 / 60.0
RENDERING_DT = 1.0 / 60.0

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
parser.add_argument("--max-steps", type=int, default=0, help="N 스텝 후 자동 종료 (0=무한)")
parser.add_argument("--target-position", type=float, nargs=3, metavar=("X", "Y", "Z"))
parser.add_argument(
    "--target-orientation",
    type=float,
    nargs=4,
    default=[0.0, 0.0, 1.0, 0.0],
    metavar=("W", "X", "Y", "Z"),
    help="목표 TCP orientation, Isaac wxyz quaternion",
)
parser.add_argument("--gripper", choices=["none", "open", "close"], default="none")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402

from isaac_indy7.camera import WristCamera  # noqa: E402
from isaac_indy7.indy7 import Indy7Gripper, Indy7IK, spawn_indy7  # noqa: E402
from isaac_indy7.ycb import print_ycb_centers, spawn_ycb  # noqa: E402


def main() -> None:
    base_position = np.asarray(INDY7_POSITION, dtype=float)

    if not os.path.isfile(INDY7_USD):
        raise FileNotFoundError(f"indy7 USD 를 찾을 수 없음: {INDY7_USD}")

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=RENDERING_DT,
    )
    world.scene.add_default_ground_plane()

    indy7 = world.scene.add(spawn_indy7(INDY7_USD, INDY7_PRIM_PATH, base_position))
    print(f"[indy7] spawned '{INDY7_USD}' -> {INDY7_PRIM_PATH} @ {base_position.tolist()}")

    ycb_paths = spawn_ycb(YCB_CONFIG, base_position=base_position)

    world.reset()
    set_camera_view(
        eye=[1.2, 1.0, 0.9],
        target=[0.35, 0.0, 0.15],
        camera_prim_path="/OmniverseKit_Persp",
    )

    ik = Indy7IK(indy7)
    gripper = Indy7Gripper(indy7)
    wrist_camera = WristCamera(CAMERA_CONFIG, parent_prim=ik.link_path("link6"), workdir=ROOT_DIR)
    wrist_camera.initialize()
    if not args.headless:
        wrist_camera.open_secondary_viewport()
    print("[ik] Indy7IK ready")
    print_ycb_centers(ycb_paths)

    target_position = None if args.target_position is None else np.asarray(args.target_position, dtype=float)
    target_orientation = np.asarray(args.target_orientation, dtype=float)
    if target_position is not None:
        print(
            f"[ik] tracking target position={target_position.tolist()}, "
            f"orientation={target_orientation.tolist()}"
        )

    if args.gripper == "open":
        gripper.open()
        gripper_hold_target = gripper.open_position
        print("[gripper] open")
    elif args.gripper == "close":
        gripper.close()
        gripper_hold_target = gripper.closed_position
        print("[gripper] close")
    else:
        gripper_hold_target = gripper.hold_position

    step = 0
    reset_needed = False
    while simulation_app.is_running():
        world.step(render=True)

        if world.is_stopped():
            reset_needed = True
        if not world.is_playing():
            continue
        if reset_needed:
            world.reset()
            ik = Indy7IK(indy7)
            gripper = Indy7Gripper(indy7)
            wrist_camera.initialize()
            if not args.headless:
                wrist_camera.open_secondary_viewport()
            reset_needed = False

        if target_position is not None:
            reachable = ik.go_to(target_position, target_orientation)
            if step % 60 == 0:
                ee_pos, ee_quat = ik.ee_pose()
                print(
                    f"[ik] step={step} reachable={reachable} "
                    f"ee_pos={np.asarray(ee_pos).round(4).tolist()} "
                    f"ee_quat={np.asarray(ee_quat).round(4).tolist()}"
                )

        gripper.hold(gripper_hold_target)
        wrist_camera.maybe_capture(step)

        step += 1
        if args.max_steps and step >= args.max_steps:
            print(f"[main] max-steps({args.max_steps}) 도달 - 종료")
            break

    simulation_app.close()


if __name__ == "__main__":
    main()
