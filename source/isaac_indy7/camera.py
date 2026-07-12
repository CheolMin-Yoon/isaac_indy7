"""Indy7 wrist camera helper."""

from __future__ import annotations

import os

import numpy as np
from isaacsim.sensors.camera import Camera

try:
    from PIL import Image

    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False


class WristCamera:
    """Create a Camera prim under the Indy7 TCP and save RGB/depth frames."""

    def __init__(self, cfg: dict, parent_prim: str, workdir: str = ".") -> None:
        self._cfg = cfg
        self._every = int(cfg.get("capture_every", 60))
        self._out_dir = os.path.join(workdir, cfg.get("output_dir", "output/camera"))
        self._rgb_dir = os.path.join(self._out_dir, "rgb")
        self._depth_dir = os.path.join(self._out_dir, "depth")
        os.makedirs(self._rgb_dir, exist_ok=True)

        self._capture_depth = bool(cfg.get("capture_depth", True))
        if self._capture_depth:
            os.makedirs(self._depth_dir, exist_ok=True)

        prim_name = cfg.get("prim_name", "zed_cam")
        self._prim_path = f"{parent_prim.rstrip('/')}/{prim_name}"
        self._camera = Camera(
            prim_path=self._prim_path,
            resolution=tuple(cfg.get("resolution", [640, 480])),
        )
        self._saved = 0

    def initialize(self) -> None:
        """Call after World.reset(), so render products are valid."""
        self._camera.initialize()
        self._camera.set_local_pose(
            translation=np.asarray(self._cfg.get("mount_translation", [0.0, 0.0, 0.08]), dtype=float),
            orientation=np.asarray(self._cfg.get("mount_orientation", [0.0, 1.0, 0.0, 0.0]), dtype=float),
            camera_axes=self._cfg.get("mount_convention", "usd"),
        )

        clip = self._cfg.get("clipping_range", [0.01, 5.0])
        self._camera.set_clipping_range(near_distance=float(clip[0]), far_distance=float(clip[1]))

        intr = self._cfg.get("intrinsics")
        if intr:
            width, _ = self._camera.get_resolution()
            h_aperture = float(intr.get("horizontal_aperture", self._camera.get_horizontal_aperture()))
            focal = float(intr["fx"]) * h_aperture / width
            self._camera.set_horizontal_aperture(h_aperture, maintain_square_pixels=True)
            self._camera.set_focal_length(focal)

        if self._capture_depth:
            self._camera.add_distance_to_image_plane_to_frame()

        try:
            k_matrix = np.asarray(self._camera.get_intrinsics_matrix())
            np.savetxt(os.path.join(self._out_dir, "cam_K.txt"), k_matrix, fmt="%.18e")
            print(
                f"[camera] mounted {self._prim_path} "
                f"fx={k_matrix[0, 0]:.2f} fy={k_matrix[1, 1]:.2f}"
            )
        except Exception:
            print(f"[camera] mounted {self._prim_path}")

    def maybe_capture(self, step: int) -> str | None:
        if self._every <= 0 or step % self._every != 0:
            return None

        rgba = self._camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return None

        rgb = np.asarray(rgba)[:, :, :3].astype(np.uint8)
        rgb_path = os.path.join(self._rgb_dir, f"{self._saved:07d}.png")
        if _HAS_PIL:
            Image.fromarray(rgb).save(rgb_path)
        else:
            rgb_path = rgb_path.replace(".png", ".npy")
            np.save(rgb_path, rgb)

        if self._capture_depth:
            depth_m = self._camera.get_depth()
            if depth_m is not None:
                depth_mm = np.nan_to_num(np.asarray(depth_m) * 1000.0, posinf=0.0, neginf=0.0)
                depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)
                depth_path = os.path.join(self._depth_dir, f"{self._saved:07d}.png")
                if _HAS_PIL:
                    Image.fromarray(depth_mm, mode="I;16").save(depth_path)
                else:
                    np.save(depth_path.replace(".png", ".npy"), depth_mm)

        self._saved += 1
        print(f"[camera] saved {rgb_path}" + (" (+depth)" if self._capture_depth else ""))
        return rgb_path

    @property
    def prim_path(self) -> str:
        return self._prim_path

    def set_as_active_viewport_camera(self) -> bool:
        """Show this camera in the active Isaac Sim viewport."""
        try:
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport is None:
                return False
            viewport.camera_path = self._prim_path
            print(f"[camera] active viewport camera -> {self._prim_path}")
            return True
        except Exception as exc:
            print(f"[camera] active viewport camera failed: {exc}")
            return False
