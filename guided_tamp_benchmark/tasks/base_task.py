#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 15.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>
import copy

import numpy as np
import pinocchio as pin
from typing import List, Tuple

from guided_tamp_benchmark.tasks.demonstration import Demonstration
from guided_tamp_benchmark.models.robots import BaseRobot
from guided_tamp_benchmark.models.objects import BaseObject, ObjectYCBV, Cuboid, Tray
from guided_tamp_benchmark.models.furniture import (
    FurnitureObject,
    Table,  # noqa F401
    Shelf,  # noqa F401
    Tunnel,  # noqa F401
)
from guided_tamp_benchmark.core import Configuration, Path

from guided_tamp_benchmark.tasks.collisions import Collision, \
    t_xyz_quat_xyzw_to_pin_se3, check_if_identity


class BaseTask:
    def __init__(
            self, task_name: str, demo_id: int, robot: BaseRobot, robot_pose_id: int
    ):
        self.task_name: str = task_name
        self.robot: BaseRobot = robot
        self.demo = Demonstration.load(
            task_name,
            demo_id=demo_id,
            robot_name=self.robot.name,
            pose_id=robot_pose_id,
        )
        self.objects = self._create_objects(self.demo.object_ids)
        self.furniture = self._create_furniture(
            self.demo.furniture_ids,
            self.demo.furniture_poses,
            self.demo.furniture_params,
        )
        self.collision = Collision(self)

    def get_robot(self) -> BaseRobot:
        """Returns the robot instance of the task"""
        return self.robot

    def get_robot_pose(self) -> np.array:
        """Returns the robot base pose as a 4x4 numpy array"""
        return self.demo.robot_pose

    def get_furniture(self) -> List[FurnitureObject]:
        """Returns the list of furniture instances"""
        return self.furniture

    def get_objects(self) -> List[BaseObject]:
        """Returns the list of object instances"""
        return self.objects

    def _check_grasp_constraint(
            self, configuration: Configuration, delta: float = 0.001
    ) -> tuple[bool, list[tuple[str]]]:
        """Check if grasp constraint is satisfied for a given @param configuration.
        It will return tuple (bool, [(str, str),...]) where bool is True if
        configuration is in grasp and list contains tuples of two string indicating the
        frames and handles that are grasped obj_name/frame_id/handle and frames and
        grippers that grasp them link/frame_id/gripper. If there is no grasp the list
        will be empty."""
        return self.collision.is_config_grasp(configuration, delta)

    def _check_place_constraint(
            self, configuration: Configuration,
            delta_upper: float = 0.002,
            delta_lower: float = -0.0001
    ) -> tuple[bool, list[tuple[str, str]]]:
        """Check if place constraint is satisfied for a given @param configuration.
        This function checks if objects in configutation are in contact. It returns
        tuple (bool, [(str, str),...]) where bool is True if configuration has contacts
        and list containing tuples of two string indicating the contact surfaces that
        are in contact obj_name/surface If there is no contacts the list will be empty.
        """
        return self.collision.is_config_placement(configuration,
                                                  delta_upper=delta_upper,
                                                  delta_lower=delta_lower)

    def _check_config_for_collision(self, configuration: Configuration) -> bool:
        """Return true if the given configuration is in collision"""
        return not self.collision.is_config_valid(configuration)

    def _check_path_for_collision(self, path: Path, delta: float) -> Tuple[bool, float]:
        """Returns tuple (Bool, t), where t is a float. Return true if configuration at
         param t is in collision. The collision will be ignored if either grasp
         constraint or placement constraint is satisfied. Collisions are check with
        pinocchio library. If there are no collisions return (False, -1). Argument delta
        is a step by which the path will be interpolated."""
        for t in np.arange(0, 1 + delta, delta):
            if not self.collision.is_config_valid(path.interpolate(t)):
                return True, t
        return False, -1

    def compute_lengths(self, path: List[Configuration]) -> Tuple[float, float, float]:
        """Compute the lengths of the path: rotation length of robot joints [rad],
        positional length of objects [m], and rotational length of objects [rad]."""
        return tuple(
            np.sum([c1.distance(c2) for c1, c2 in zip(path[:-1], path[1:])], axis=0)
        )

    def path_is_successful(self,
                           path: List[Configuration],
                           error_robot_distance: float = 0.0001,
                           error_identity: float = 0.001,
                           error_placement_upper: float = 0.002,
                           error_placement_lower: float = -0.0001,
                           error_grasp: float = 0.001
                           ) -> Tuple[bool, str]:
        """Return true if path solves the given task."""

        prev_placed = []
        prev_config = []
        objects = self.objects
        for i, obj in enumerate(objects):
            if obj.name == "tray":
                objects.pop(i)

        init_config = self.collision.separate_configs(
            Configuration(self.robot.initial_configuration(),
                          self.demo.subgoal_objects_poses[:, 0]))
        goal_config = self.collision.separate_configs(
            Configuration(self.robot.initial_configuration(),
                          self.demo.subgoal_objects_poses[:, -1]))

        first_config = self.collision.separate_configs(path[0])
        last_config = self.collision.separate_configs(path[-1])

        for o in objects:
            if not check_if_identity(init_config[o.name], first_config[o.name],
                                     error=error_identity):
                return False, f"first pose of object {o.name} doesn't match with " \
                              f"its initial configuration from demonstration"
            if not check_if_identity(goal_config[o.name], last_config[o.name],
                                     error=error_identity):
                return False, f"last pose of object {o.name} doesn't match with " \
                              f"its goal configuration from demonstration"

        if \
        self.compute_lengths([Configuration(init_config[self.robot.name], [np.eye(4)]),
                              Configuration(first_config[self.robot.name],
                                            [np.eye(4)])])[0] > error_robot_distance:
            return False, f"first configuration of robot {self.robot.name} doesn't" \
                          f" match with its initial configuration"

        if \
        self.compute_lengths([Configuration(goal_config[self.robot.name], [np.eye(4)]),
                              Configuration(last_config[self.robot.name],
                                            [np.eye(4)])])[0] > error_robot_distance:
            return False, f"last configuration of robot {self.robot.name} doesn't" \
                          f" match with its goal configuration"

        for i, c in enumerate(path):
            if self._check_config_for_collision(c):
                return False, f"collision at config {i}"

            grasp = self._check_grasp_constraint(c, delta=error_grasp)
            place = self._check_place_constraint(c, delta_upper=error_placement_upper,
                                                 delta_lower=error_placement_lower)

            is_constrained = len(objects) * [False]
            is_placed = []

            for p in place[1]:
                for j, o in enumerate(objects):
                    if p[1].find(o.name) != -1:
                        is_constrained[j] = True
                    else:
                        is_placed.append(o.name)

            for g in grasp[1]:
                for j, o in enumerate(objects):
                    if g[1].find(o.name) != -1:
                        is_constrained[j] = True

            if not all(is_constrained):
                for j, ic in enumerate(is_constrained):
                    if ic is False:
                        return False, f"unconstrained object {objects[j].name}" \
                                      f" at config {i}"

            curr_config = self.collision.separate_configs(c)
            for pp in prev_placed:
                for ip in is_placed:
                    if pp == ip:
                        if not check_if_identity(prev_config[pp], curr_config[ip],
                                                 error=error_identity):
                            return False, f"object {pp}, moved between configuration " \
                                          f"{i} and {i - 1}, even though its under" \
                                          f"placement constraint."
            prev_placed = is_placed
            prev_config = curr_config

        return True, "path is successful"

    def compute_n_grasps(self, path: List[Configuration]) -> int:
        """Compute the amount of grasp-release actions."""
        grasps, previous_grasps = 0, []
        for c in path:
            _, current_grasp = self._check_grasp_constraint(c)
            tmp = len(current_grasp)
            for pg in previous_grasps:
                if len(current_grasp) == 0:
                    tmp = 0
                for cg in current_grasp:
                    if pg == cg:
                        tmp -= 1
            previous_grasps = copy.deepcopy(current_grasp)
            grasps += tmp

        return grasps

    @staticmethod
    def _create_objects(obj_ids) -> List[BaseObject]:
        """Utility function that converts text representation of objects into the object
        instances."""
        obj = []
        for o in obj_ids:
            if o[:4].lower() == "ycbv":
                obj.append(ObjectYCBV("obj_0000" + o[5:]))
            elif o[:6].lower() == "cuboid":
                tmp = o.split("_")
                obj.append(Cuboid([float(tmp[1]), float(tmp[2]), float(tmp[3])]))
            elif o[:4].lower() == "tray":
                tmp = o.split("_")
                obj.append(
                    Tray(tray_size=[float(tmp[1]), float(tmp[2]), float(tmp[3])])
                )
            else:
                raise ValueError(f"Unknown object {o}")
        return obj

    @staticmethod
    def _create_furniture(fur_id, fur_poses, fur_params) -> List[FurnitureObject]:
        """Utility function that converts a text representation of furniture object into
        furniture instances."""
        return [
            globals()[f.capitalize()](pose=pose, **param)
            for f, pose, param in zip(fur_id, fur_poses, fur_params)
        ]
