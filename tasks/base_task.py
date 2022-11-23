#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 15.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import numpy as np
from typing import Optional, List, Tuple
from pinocchio.pinocchio_pywrap.rpy import matrixToRpy

from tasks.demonstration import Demonstration
from models.robots.base import BaseRobot
from models.objects import *
from models.furniture import *


class BaseTask:
    def __init__(self, task_name: str, robot: BaseRobot):
        self.task_name: str = task_name
        self.demo: Optional[Demonstration] = None
        self.robot: BaseRobot = robot
        self.furniture: List[FurnitureObject] = []
        self.objects: List[BaseObject] = []

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

    def _check_grasp_constraint(self, q: np.array) -> bool:
        """ Check if grasp constraint is satisfied for a given configuration q."""
        pass

    def _check_place_constraint(self, q: np.array) -> bool:
        """ Check if place constraint is satisfied for a given configuration q."""
        pass

    def _check_collision(self, path: np.array) -> bool:
        """Return true if every configuration of the path is collision-free. Collisions are check with hpp-fcl library.
         Path size is Tx[N+7*M], where T is a number of timesteps, N is a number of DoFs of a Robot and
         M is a number of objects. Each object pose is represented by 3 position and 4 quaternion values"""
        pass

    def compute_lengths(self, path: np.array) -> Tuple[float, float, float]:
        """Compute the lengths of the path: rotation length of robot joints [rad], positional length of objects [m],
        and rotational length of objects [rad]. """
        pass

    def path_is_successful(self, path: np.array) -> bool:
        """Return true if path solves the given task."""
        pass

    def compute_n_grasps(self, path: np.array) -> int:
        """Compute the amount of grasp-release actions."""
        pass

    def load_demo(self, demo_id: str, pose_id: int):
        """this function loads demo of given id for current task and will create objects and furniture instances"""
        self.demo = Demonstration.load(self.task_name, demo_id=demo_id, robot_name=self.robot.name, pose_id=pose_id)
        self.objects = self._create_objects(self.demo.object_ids)
        self.furniture = self._create_furniture(self.demo.furniture_ids, self.demo.furniture_poses,
                                                self.demo.furniture_param)

    @staticmethod
    def _create_objects(obj_ids):
        """Utility function that converts text representation of objects into the object instances. """
        obj = []
        for o in obj_ids:
            if o[:4] == "ycbv":
                obj.append(ObjectYCBV("obj_0000" + o[5:]))
            elif o[:6] == "cuboid":
                tmp = o.split("_")
                obj.append(Cuboid([float(tmp[1]), float(tmp[2]), float(tmp[3])]))
            else:
                raise ValueError(f"Unknown object {o}")
        return obj

    @staticmethod
    def _create_furniture(fur_id, fur_poses, fur_params):
        """Utility function that converts a text representation of furniture object into furniture instances."""
        return [
            globals()[f.capitalize()](position=pose[:3, 0:3], rpy=matrixToRpy(pose[:3, :3]).tolist(), **param) for
            f, pose, param in zip(fur_id, fur_poses, fur_params)
        ]
