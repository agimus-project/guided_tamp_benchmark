#!/usr/bin/env python
import numpy as np

# Copyright (c) CTU -- All Rights Reserved
# Created on: 15.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from tasks.demonstration import Demonstration

from models.robots.base import BaseRobot

from models.objects.base import BaseObject
from models.objects.cuboid import Cuboid
from models.objects.object_ycbv import ObjectYCBV

from models.furniture.base import FurnitureObject
from models.furniture.shelf import Shelf
from models.furniture.table import Table
from models.furniture.tunnel import Tunnel

from scipy.spatial.transform import Rotation as R

from typing import Optional, List, Tuple


class BaseTask:
    def __init__(self, task_name: str, robot: BaseRobot):
        self.task_name: str = task_name
        self.demo: Optional[Demonstration] = None
        self.robot: BaseRobot = robot
        self.furniture: List[FurnitureObject] = []
        self.objects: List[BaseObject] = []

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

    @staticmethod
    def _create_objects(obj_ids):
        """Utility function that converts a text representation of objects into object instances. """
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
    def _create_furniture(fur_id, fur_poses):
        """Utility function that converts a text representation of furniture object into furniture instances.
         TODO: this function is not complete!"""
        fur = []
        for f in fur_id:
            if f == "table":
                fur.append(Table())
            elif f == "shelf":
                fur.append(Shelf())
            elif f == "tunnel":
                fur.append(Tunnel())
            else:
                raise ValueError(f"Unknown furniture {f}")
        return fur
