"""Indy7 spawn, IK, and gripper helpers."""

from __future__ import annotations

import numpy as np


def spawn_indy7(usd_path: str, prim_path: str, position):
    """Add an Indy7 USD reference and wrap it as a SingleArticulation.

    The combined asset USDs (gripper/camera variants) nest the actual
    ArticulationRootAPI prim one or more levels below their defaultPrim, so
    ``prim_path`` itself is never the articulation root after referencing —
    the root is found by scanning the referenced subtree instead of assuming
    it sits exactly at ``prim_path``.
    """
    from pxr import Usd, UsdPhysics

    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage

    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    stage = get_current_stage()
    root_prim = stage.GetPrimAtPath(prim_path)
    articulation_prim = next(
        (p for p in Usd.PrimRange(root_prim) if p.HasAPI(UsdPhysics.ArticulationRootAPI)),
        None,
    )
    if articulation_prim is None:
        raise RuntimeError(f"No ArticulationRootAPI prim found under {prim_path} in {usd_path}")

    return SingleArticulation(
        prim_path=str(articulation_prim.GetPath()),
        name="indy7",
        position=np.asarray(position, dtype=float),
    )


class Indy7IK:
    """PINK differential IK wrapper for the Indy7 TCP."""

    def __init__(self, indy7, ee_link_name: str = "tcp", num_arm_dofs: int = 6) -> None:
        import omni.kit.app

        ext_mgr = omni.kit.app.get_app().get_extension_manager()
        ext_mgr.set_extension_enabled_immediate("isaacsim.robot_motion.pink", True)
        ext_mgr.set_extension_enabled_immediate("isaacsim.replicator.teleop", True)

        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.replicator.teleop.controllers.pink_ik import (
            PinkIKController as TeleopPinkIKController,
        )

        available, reason = TeleopPinkIKController.get_backend_status()
        if not available:
            raise ImportError(reason)

        exp_robot = Articulation(indy7.prim_path)
        link_names = exp_robot.link_names
        ee_link_index = link_names.index(ee_link_name) if ee_link_name in link_names else 0
        link_paths = getattr(exp_robot, "link_paths", None)
        ee_path = str(link_paths[0][ee_link_index]) if link_paths is not None else f"{indy7.prim_path}/{ee_link_name}"
        ee_link = RigidPrim(ee_path)

        self._num_arm_dofs = num_arm_dofs
        self._ee_path = ee_path
        self._link_names = link_names
        self._link_paths = link_paths
        self._robot_prim_path = indy7.prim_path
        self._articulation_controller = indy7.get_articulation_controller()

        robot_root_name = ee_path.rstrip("/").rsplit("/", 2)[-2]
        qualified_ee_link_name = f"{robot_root_name}_{ee_link_name}"

        self._controller = TeleopPinkIKController(
            robot=exp_robot,
            ee_link=ee_link,
            ee_link_index=ee_link_index,
            num_arm_dofs=num_arm_dofs,
            ee_link_name=qualified_ee_link_name,
            articulation_path=indy7.prim_path,
            position_cost=0.5,
            orientation_cost=1.0,
            posture_cost=1e-3,
            solver="osqp",
        )

    @property
    def ee_path(self) -> str:
        return self._ee_path

    def link_path(self, link_name: str) -> str:
        """Resolve any articulation link's prim path by name (e.g. "link6")."""
        if link_name in self._link_names:
            index = self._link_names.index(link_name)
            if self._link_paths is not None:
                return str(self._link_paths[0][index])
        return f"{self._robot_prim_path}/{link_name}"

    def go_to(self, position, orientation) -> bool:
        """Apply one differential IK step toward a world-frame pose.

        `position` is xyz in meters. `orientation` is Isaac wxyz quaternion.
        """
        from isaacsim.core.utils.types import ArticulationAction

        w, x, y, z = np.asarray(orientation, dtype=float)
        self._controller.set_target(
            tuple(np.asarray(position, dtype=float)),
            (x, y, z, w),
        )

        new_arm_q = self._controller.compute()
        if new_arm_q is None:
            return False

        self._articulation_controller.apply_action(
            ArticulationAction(
                joint_positions=new_arm_q,
                joint_indices=list(range(self._num_arm_dofs)),
            )
        )
        return bool(self._controller.reachable)

    def reset(self) -> None:
        self._controller.reset()

    def ee_pose(self):
        from isaacsim.core.prims import SingleXFormPrim

        return SingleXFormPrim(self._ee_path).get_world_pose()


class Indy7Gripper:
    """Simple position control for the attached Robotiq-style finger joint."""

    open_position = 0.0
    closed_position = 0.7

    def __init__(self, indy7, joint_name: str = "finger_joint") -> None:
        self._indy7 = indy7
        self._joint_name = joint_name
        self._joint_index = self._find_joint_index(indy7.dof_names, joint_name)
        if self._joint_index is None:
            print(f"[gripper] joint '{joint_name}' not found; available={indy7.dof_names}")
        else:
            print(f"[gripper] joint '{joint_name}' @ index {self._joint_index}")

    @staticmethod
    def _find_joint_index(dof_names: list[str], joint_name: str) -> int | None:
        if joint_name in dof_names:
            return dof_names.index(joint_name)
        return None

    @property
    def available(self) -> bool:
        return self._joint_index is not None

    def open(self) -> None:
        self.set(self.open_position)

    def close(self) -> None:
        self.set(self.closed_position)

    def set(self, target: float) -> None:
        if self._joint_index is None:
            return
        from isaacsim.core.utils.types import ArticulationAction

        self._indy7.apply_action(
            ArticulationAction(
                joint_positions=[float(target)],
                joint_indices=[self._joint_index],
            )
        )

    @property
    def hold_position(self) -> float:
        if self._joint_index is None:
            return self.open_position
        positions = self._indy7.get_joint_positions()
        return float(positions[self._joint_index])

    def hold(self, target: float | None = None) -> None:
        self.set(self.hold_position if target is None else target)
