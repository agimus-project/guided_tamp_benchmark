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
        """ Check if grasp constraint is satisfied for a given configuration q."""
        pass

    def _check_collision(self, path: np.array) -> bool:
        """Return true if objects are not in collision, i.e. configurations are valid. Path if of size Tx[N+7*M]
         where N is number of DoFs of Robot and M is number of objects. Each object is represented by 3 positions and
         4 quaternion values"""
        pass

    def compute_lengths(self, path: np.array) -> Tuple[float, float, float]:
        """Compute the lengths of the path: rotation length of robot joints [rad], positional length of objects [m],
        and rotational length of objects [rad]. """
        pass

    def is_successful(self, path: np.array) -> bool:
        """Return true if path solve the given task."""
        pass

    def compute_n_grasps(self, config_list):
        pass

    @staticmethod
    def _create_objects(obj_ids):
        """Utility function that converts text representation of objects into the actual object instances. """
        obj = []
        for o in obj_ids:
            if o[:4] == "ycbv":
                obj.append(ObjectYCBV("obj_0000" + o[5:]))
            elif o[:6] == "cuboid":
                tmp = o.split("_")
                obj.append(Cuboid([float(tmp[1]), float(tmp[2]), float(tmp[3])]))
        return obj

    @staticmethod
    def _create_furniture(fur_id, fur_poses, fur_param):
        """Create furniture objects from the text description."""
        fur = []
        for i, f in enumerate(fur_id):
            if f == "table":
                r = R.from_matrix(fur_poses[i,:3,:3])
                rpy = r.as_euler("xyz", degrees=False).tolist()
                fur.append(Table(position=fur_poses[0, :3, 2:3],rpy=rpy,desk_size=fur_param[i]))
            elif f == "shelf":
                r = R.from_matrix(fur_poses[i, :3, :3])
                rpy = r.as_euler("xyz", degrees=False).tolist()
                fur.append(Shelf(position=fur_poses[0, :3, 2:3],rpy=rpy, display_inside_shelf=fur_param[i]))
            elif f == "tunnel":
                r = R.from_matrix(fur_poses[i, :3, :3])
                rpy = r.as_euler("xyz", degrees=False).tolist()
                fur.append(Tunnel(position=fur_poses[0, :3, 2:3],rpy=rpy, lengths=fur_param[i][0],
                                  tunnel_walls_thickness=fur_param[i][1], collision_walls_thickness=fur_param[i][2],
                                  walls_display=True))
        return fur
