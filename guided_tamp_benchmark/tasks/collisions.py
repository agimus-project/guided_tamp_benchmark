#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 20.02.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import pinocchio as pin
import sys
import copy
import numpy as np

from typing import List, Tuple

from guided_tamp_benchmark.core import Configuration

from guided_tamp_benchmark.models.robots.base import BaseRobot
from guided_tamp_benchmark.models.objects.base import BaseObject
from guided_tamp_benchmark.models.furniture.base import FurnitureObject

from guided_tamp_benchmark.models.utils import get_models_data_directory


def _rename_frames(model: pin.Model, model_name: str):
    """Renames the frame names in given model to {previous name}_{model_name}_{i}."""
    for i, frame in enumerate(model.frames):
        if i == 0:
            continue
        frame.name = f"{frame.name}_{model_name}"


def _rename_joints(model: pin.Model, model_name: str):
    """Renames the joint names in given model to {previous name}_{model_name}_{i}."""
    for i in range(0, len(model.names)):
        if i == 0:
            continue
        model.names[i] = f"{model.names[i]}_{model_name}"


def _rename_geometry(collision_model: pin.GeometryModel, model_name: str):
    """Renames the joint names in given model to {previous name}_{model_name}_{i}."""
    for i, geom in enumerate(collision_model.geometryObjects):
        geom.name = f"{geom.name[0:-2]}_{model_name}"


def _remove_collisions_for_tunnel(
        full_coll_mod: pin.GeometryModel,
        rob_coll_mod: pin.GeometryModel,
        disabled_tunnel_links: List[str],
):
    """removes collision pairs between all robot links and
    Tunnel.disabled_robot_collision_for_links"""
    disabled_id = []
    for i, obj in enumerate(full_coll_mod.geometryObjects):
        for disabled in disabled_tunnel_links:
            if obj.name.find(disabled) != -1:
                disabled_id.append(i)

    for obj in rob_coll_mod.geometryObjects:
        for id in disabled_id:
            full_coll_mod.removeCollisionPair(
                pin.CollisionPair(full_coll_mod.getGeometryId(obj.name), id)
            )


def t_xyz_quat_xyzw_to_pin_se3(pose: list) -> pin.SE3:
    """returns pinocchio SE3 pose from pose made of translation xyz and
    quaternion xyzw"""
    return pin.XYZQUATToSE3(pose[:3] + pose[-3:] + [pose[-4]])


def check_if_identity(pose1: list, pose2: list, error: float = 0.0001) -> bool:
    """check if transformation from pose1 to pose2 is identity with given error"""
    T_o_p1, T_2p_o = pin.XYZQUATToSE3(pose1), pin.XYZQUATToSE3(pose2).inverse()
    T_2p_p1 = T_2p_o * T_o_p1
    return T_2p_p1.isIdentity(prec=error)


def find_frame_in_frames(model: pin.Model, frame: str) -> int:
    """Will find given frame name or partial frame name in the frames of given pinocchio
    model."""
    for i, f in enumerate(model.frames):
        if f.name.find(frame) != -1:
            return i
    return -1


def _create_model(
        robots: List[BaseRobot],
        objects: List[BaseObject],
        furniture: List[FurnitureObject],
        robot_poses: List[pin.SE3],
        remove_tunnel_collisions: bool,
) -> (pin.Model, pin.GeometryModel):
    """Creates pinocchio urdf model and pinocchio collision model from given robots,
    furniture and objects."""
    p_r = None
    c_r = None

    for i, model in enumerate(reversed(furniture + objects)):
        if i < len(objects):
            p, c, _ = pin.buildModelsFromUrdf(
                model.urdfFilename,
                package_dirs=str(get_models_data_directory()),
                root_joint=pin.JointModelFreeFlyer(),
            )
        else:
            p, c, _ = pin.buildModelsFromUrdf(
                model.urdfFilename, package_dirs=str(get_models_data_directory())
            )

        _rename_joints(p, model.name + f"_{i}")
        _rename_frames(p, model.name + f"_{i}")
        _rename_geometry(c, model.name + f"_{i}")
        c.addAllCollisionPairs()
        if i == 0:
            p_r = p
            c_r = c
            continue
        p_r, c_r = pin.appendModel(
            p_r, p, c_r, c, 0, pin.SE3(np.eye(3), np.array([0, 0, 0]))
        )

    for i, robot in enumerate(robots):
        p, c, _ = pin.buildModelsFromUrdf(
            robot.urdfFilename, package_dirs=str(get_models_data_directory())
        )
        c.addAllCollisionPairs()
        pin.removeCollisionPairs(p, c, robot.srdfFilename)
        _rename_joints(p, robot.name + f"_{i}")
        _rename_frames(p, robot.name + f"_{i}")
        _rename_geometry(c, robot.name + f"_{i}")
        p_r, c_r = pin.appendModel(p_r, p, c_r, c, 0, robot_poses[i])
        if remove_tunnel_collisions:
            for i, model in enumerate(reversed(furniture)):
                if model.name == "tunnel":
                    suffix = "_" + model.name + f"_{i + len(objects)}"
                    _remove_collisions_for_tunnel(
                        c_r,
                        c,
                        [s + suffix for s in model.disabled_collision_links_for_robot],
                    )
    return p_r, c_r


def _extract_from_task(task) -> dict:
    """Extracts robots, objects, furniture and robot poses from task and returns it in
    dictionary"""
    furniture = task.furniture
    objects = task.objects
    for i, obj in enumerate(objects):
        if obj.name == "tray":
            objects.pop(i)

    # TODO: change for multi robot
    if isinstance(task.robot, list):
        robots = task.robot
    else:
        robots = [task.robot]
    robot_poses = [
        pin.SE3(
            task.get_robot_pose()[:3, :3], np.squeeze(task.get_robot_pose()[:3, 3:])
        )
    ]

    return {
        "robots": robots,
        "objects": objects,
        "furniture": furniture,
        "robot_poses": robot_poses,
    }


def pose_as_matrix_to_pose_as_quat(pose: np.ndarray) -> np.ndarray:
    T = pin.SE3(pose[:3, :3], np.squeeze(pose[:3, 3:]))
    xyz_quat = pin.se3ToXYZQUAT(T)
    return xyz_quat


def convex_shape(
        shape_points: np.ndarray, normal: np.ndarray, frame: pin.SE3
) -> Tuple[np.ndarray, np.ndarray]:
    """will create the equiation for convex shape in form of Ax>=b, where A is matrix
    and b vector."""
    A, b = [], []
    normal_transformed = frame.rotation @ normal
    # normal = frame.homogeneous @ normal
    for x, p in enumerate(shape_points):
        p_a, p_b = np.resize(p, 4), np.resize(shape_points[x - 1], 4)
        p_a[-1], p_b[-1] = 1, 1
        p_a, p_b = frame.homogeneous @ p_a, frame.homogeneous @ p_b
        a = np.cross(normal_transformed[:3], (p_a - p_b)[:3])
        A.append(a[:2])
        b.append(np.dot(np.array(A[x]), p_b[:2]))
    return np.array(A), np.array(b)


def ortonormalization(
        n: np.ndarray, y: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """creates 3 orthonormal vector. Vector n is the result between cross product x and
    y.
     Thus, n and y are orthogonal."""
    q1 = n / np.linalg.norm(n)
    q2 = y / np.linalg.norm(y)
    q3 = np.cross(y, n) / np.linalg.norm(np.cross(y, n))
    return q3, q2, q1


class Collision:
    """The collision class consists of Pinocchio urdf and collision models and functions
    for collision checking and Pinocchio model rendering."""

    def __init__(self, task):
        """Initilizie with task eg. ShelfTask..."""
        self.task = task
        self.pin_mod, self.col_mod = _create_model(
            remove_tunnel_collisions=task.task_name == "tunnel",
            **_extract_from_task(task),
        )
        self.data = self.pin_mod.createData()

    def is_config_valid(self, configuration: Configuration) -> bool:
        """Returns true if given configuration is collision free"""
        config = configuration.to_numpy()

        # Create data structures
        geom_data = pin.GeometryData(self.col_mod)

        # Compute all the collisions
        if pin.computeCollisions(
                self.pin_mod, self.data, self.col_mod, geom_data, config, False
        ):
            for k in range(len(self.col_mod.collisionPairs)):
                cr = geom_data.collisionResults[k]
                cp = self.col_mod.collisionPairs[k]
                print(
                    "collision pair:",
                    cp.first,
                    ",",
                    cp.second,
                    "- collision:",
                    "Yes" if cr.isCollision() else "No",
                )
            return False
        else:
            return True

    def is_config_placement(
            self,
            configuration: Configuration,
            delta_upper: float = 0.002,
            delta_lower: float = -0.0001,
    ) -> tuple[bool, list[tuple[str, str]]]:
        """This function checks if objects in configutation are in contact. It returns
        tuple (bool, [(str, str),...]) where bool is True if configuration has contacts
        and list containing tuples of two string indicating the contact surfaces that
        are in contact obj_name/surface If there is no contacts the list will be empty.
        """

        def find_info_for_contact_surface(
                contacts: np.ndarray,
        ) -> Tuple[np.ndarray, np.ndarray, pin.SE3]:
            # finds the highest cross product in contact surface shape
            x, y = 0, 0
            for i in range(len(contacts)):
                for j in range(i + 1, len(contacts)):
                    if np.linalg.norm(
                            np.cross(contacts[i] - contacts[0],
                                     contacts[j] - contacts[0])
                    ) > np.linalg.norm(
                        np.cross(contacts[x] - contacts[0], contacts[y] - contacts[0])
                    ):
                        x, y = i, j

            n = np.cross(
                contacts[x] - contacts[0], contacts[y] - contacts[0]
            ) / np.linalg.norm(
                np.cross(contacts[x] - contacts[0], contacts[y] - contacts[0])
            )
            base = np.array(ortonormalization(n, contacts[y] - contacts[0]))
            T_fl_fp = pin.SE3(base, contacts[0])
            A, b = convex_shape(contacts, n, T_fl_fp.inverse())

            return A, b, T_fl_fp

        def are_points_in_convex_shape(
                A: np.ndarray, b: np.ndarray, points: np.ndarray, T: pin.SE3, d_u, d_l
        ) -> bool:
            """chceck whether the points satisfy the convex equation Ax>=b, and whether
            they are in distance d that satisfies d_l < d < d_u"""
            tmp = 0
            for x in range(len(points)):
                T_ol_op = pin.SE3(np.eye(3), points[x])
                T_fp_op = T * T_ol_op
                o_shapes.append(T_fp_op.translation)
                if d_l < T_fp_op.translation[-1] < d_u:
                    tmp = (
                        tmp + 1
                        if sum(A @ T_fp_op.translation[:2] >= b) == len(b)
                        else tmp
                    )

            return tmp == len(points)

        pin.forwardKinematics(self.pin_mod, self.data, configuration.to_numpy())
        pin.updateFramePlacements(self.pin_mod, self.data)

        task_info = _extract_from_task(self.task)
        robots = task_info["robots"]
        objects = task_info["objects"]
        furniture = task_info["furniture"]

        contacts = []
        for i, f in enumerate(furniture[::-1] + robots):
            f_contacts = f.get_contacts_info()
            for k_fc in f_contacts:
                if i < len(furniture):
                    T_o_fl = self.data.oMf[
                        find_frame_in_frames(
                            self.pin_mod,
                            f_contacts[k_fc]["link"] + f"_{f.name}_{i + len(objects)}",
                        )
                    ]
                else:
                    T_o_fl = self.data.oMf[
                        find_frame_in_frames(
                            self.pin_mod,
                            f_contacts[k_fc]["link"]
                            + f"_{f.name}_{i - len(furniture)}",
                        )
                    ]
                for j in range(len(f_contacts[k_fc]["shapes"])):
                    A, b, T_fl_fp = find_info_for_contact_surface(
                        f_contacts[k_fc]["shapes"][j]
                    )
                    for k, o in enumerate(reversed(objects)):
                        objects_contacts = o.get_contacts_info()
                        for k_oc in objects_contacts:
                            T_o_ol = self.data.oMf[
                                find_frame_in_frames(
                                    self.pin_mod,
                                    objects_contacts[k_oc]["link"] + f"_{o.name}_{k}",
                                )
                            ]
                            for m in range(len(objects_contacts[k_oc]["shapes"])):
                                o_shapes = []
                                T_fp_ol = T_fl_fp.inverse() * T_o_fl.inverse() * T_o_ol
                                if are_points_in_convex_shape(
                                        A,
                                        b,
                                        objects_contacts[k_oc]["shapes"][m],
                                        T_fp_ol,
                                        delta_upper,
                                        delta_lower,
                                ):
                                    contacts.append(
                                        (f"{f.name}/{k_fc}", f"{o.name}/{k_oc}")
                                    )
        if len(contacts) > 0:
            return True, contacts
        else:
            return False, []

    def is_config_grasp(
            self, configuration: Configuration, delta: float = 0.001
    ) -> Tuple[bool, list]:
        """This function will check if configuration is in grasp or not. It will return
        tuple (bool, [(str, str),...]) where bool is True if configuration is in grasp
        and list contains tuples of two string indicating the frames and handles that
        are grasped obj_name/frame_id/handle and frames and grippers that grasp them
         link/frame_id/gripper. If there is no grasp the list will be empty."""
        pin.forwardKinematics(self.pin_mod, self.data, configuration.to_numpy())
        pin.updateFramePlacements(self.pin_mod, self.data)

        # t = data.oMf[self.pin_mod.getFrameId("")

        task_info = _extract_from_task(self.task)
        robots = task_info["robots"]
        objects = task_info["objects"]

        list_of_grasps = []
        for i, r in enumerate(robots):
            grippers = r.get_grippers_info()
            for k_g in grippers:
                frame = grippers[k_g]["link"] + f"_{r.name}_{i}"
                rob_frame_id = find_frame_in_frames(self.pin_mod, frame)
                assert rob_frame_id != -1, (
                    f"frame {frame} isn't inbetween the frames of the given"
                    f" pinocchio model"
                )
                T_o_lr = self.data.oMf[rob_frame_id]
                T_lr_g = t_xyz_quat_xyzw_to_pin_se3(grippers[k_g]["pose"])
                T_o_g = T_o_lr * T_lr_g
                for j, o in enumerate(reversed(objects)):
                    handles = o.get_handles_info()
                    for k_h in handles:
                        frame = handles[k_h]["link"] + f"_{o.name}_{j}"
                        obj_frame_id = find_frame_in_frames(self.pin_mod, frame)
                        assert obj_frame_id != -1, (
                            f"frame {frame} isn't inbetween the frames of "
                            f"the given pinocchio model"
                        )
                        T_o_lo = self.data.oMf[obj_frame_id]
                        T_lo_h = t_xyz_quat_xyzw_to_pin_se3(handles[k_h]["pose"])
                        T_o_h = T_o_lo * T_lo_h

                        if (
                                np.linalg.norm(pin.log(T_o_g.inverse() * T_o_h)) < delta
                                and handles[k_h]["clearance"] <= grippers[k_g][
                            "clearance"]
                        ):
                            list_of_grasps.append(
                                (
                                    f"{r.name}_{i}/{rob_frame_id}/{k_g}",
                                    f"{o.name}_{j}/{obj_frame_id}/{k_h}",
                                )
                            )

        if len(list_of_grasps) > 0:
            return True, list_of_grasps
        else:
            return False, []

    def separate_configs(self, configuration: Configuration) -> dict:
        separated = {}
        configs = configuration.to_numpy()
        task_info = _extract_from_task(self.task)
        robots, objects = task_info["robots"], task_info["objects"]

        sum = 0
        for i, r in enumerate(robots):
            separated[r.name] = configs[sum: sum + len(r.initial_configuration())]
            sum += len(r.initial_configuration())

        for i, o in enumerate(reversed(objects)):
            separated[o.name] = configs[sum + i * 7:sum + i * 7 + 7]

        return separated

    def visualize_through_pinocchio(self, configuration: Configuration):
        """will visualize the given configuration on Pinocchio collision model"""
        from pinocchio.visualize import MeshcatVisualizer

        config = configuration.to_numpy()
        viz = MeshcatVisualizer(self.pin_mod, self.col_mod, self.col_mod)

        # Initialize the viewer.
        try:
            viz.initViewer(open=True)
        except ImportError as error:
            print(error)
            sys.exit(0)

        try:
            viz.loadViewerModel("shapes")
        except AttributeError as error:
            print(error)
            sys.exit(0)

        viz.display(config)
        input("press enter to continue")


if __name__ == "__main__":
    from guided_tamp_benchmark.models.robots import KukaMobileIIWARobot
    from guided_tamp_benchmark.tasks import WaiterTask

    task = WaiterTask(demo_id=0, robot=KukaMobileIIWARobot(), robot_pose_id=0)

    collision = Collision(task)

    config = Configuration(
        task.robot.initial_configuration(), task.demo.objects_poses[:3, 0]
    )

    collision.is_config_valid(config)
    collision.visualize_through_pinocchio(config)
    pass
