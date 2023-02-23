#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 15.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import numpy as np
from typing import List, Tuple

from guided_tamp_benchmark.tasks.demonstration import Demonstration
from guided_tamp_benchmark.models.robots import BaseRobot
from guided_tamp_benchmark.models.objects import *
from guided_tamp_benchmark.models.furniture import *
from .configuration import Configuration


class BaseTask:
    def __init__(self, task_name: str, demo_id: int, robot: BaseRobot, robot_pose_id: int):
        self.task_name: str = task_name
        self.robot: BaseRobot = robot
        self.demo = Demonstration.load(task_name, demo_id=demo_id, robot_name=self.robot.name, pose_id=robot_pose_id)
        self.objects = self._create_objects(self.demo.object_ids)
        self.furniture = self._create_furniture(self.demo.furniture_ids, self.demo.furniture_poses,
                                                self.demo.furniture_params)

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

    def _check_grasp_constraint(self, configuration: Configuration) -> bool:
        """ Check if grasp constraint is satisfied for a given @param configuration."""
        pass

    def _check_place_constraint(self, configuration: Configuration) -> bool:
        """ Check if place constraint is satisfied for a given @param configuration."""
        pass

    def _check_collision(self, path: List[Configuration]) -> bool:
        """Return true if every configuration of the path is collision-free.
         Collisions are check with pinocchio library.
         """

        pass

    def compute_lengths(self, path: List[Configuration]) -> Tuple[float, float, float]:
        """Compute the lengths of the path: rotation length of robot joints [rad], positional length of objects [m],
        and rotational length of objects [rad]. """
        return tuple(np.sum([c1.distance(c2) for c1, c2 in zip(path[:-1], path[1:])], axis=0))

    def path_is_successful(self, path: List[Configuration]) -> bool:
        """Return true if path solves the given task."""
        pass

    def compute_n_grasps(self, path: List[Configuration]) -> int:
        """Compute the amount of grasp-release actions."""
        pass

    @staticmethod
    def _create_objects(obj_ids) -> List[BaseObject]:
        """Utility function that converts text representation of objects into the object instances. """
        obj = []
        for o in obj_ids:
            if o[:4].lower() == "ycbv":
                obj.append(ObjectYCBV("obj_0000" + o[5:]))
            elif o[:6].lower() == "cuboid":
                tmp = o.split("_")
                obj.append(Cuboid([float(tmp[1]), float(tmp[2]), float(tmp[3])]))
            elif o[:4].lower() == "tray":
                tmp = o.split("_")
                obj.append(Tray(tray_size=[float(tmp[1]), float(tmp[2]), float(tmp[3])]))
            else:
                raise ValueError(f"Unknown object {o}")
        return obj

    @staticmethod
    def _create_furniture(fur_id, fur_poses, fur_params) -> List[FurnitureObject]:
        """Utility function that converts a text representation of furniture object into furniture instances."""
        return [globals()[f.capitalize()](pose=pose, **param) for f, pose, param in zip(fur_id, fur_poses, fur_params)]
