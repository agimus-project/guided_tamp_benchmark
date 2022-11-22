#!/usr/bin/env python

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

from typing import Optional, List


class BaseTask:
    def __init__(self, task_name, robot):
        self.task_name: Optional[str] = task_name
        self.demo: Optional[Demonstration] = None
        self.robot: Optional[BaseRobot] = robot
        self.furniture: Optional[List[FurnitureObject]] = None
        self.objects: Optional[List[BaseObject]] = None

    def get_robot(self):
        pass

    def get_furniture(self):
        pass

    def get_objects(self):
        pass

    def check_collision(self):
        pass

    def check_grasp_constraint(self):
        pass

    def check_place_constraint(self):
        pass

    def compute_length(self):
        pass

    def is_successful(self):
        pass

    def compute_n_grasps(self, config_list):
        pass

    @staticmethod
    def create_objects(obj_ids):
        obj = []
        for o in obj_ids:
            if o[:4] == "ycbv":
                obj.append(ObjectYCBV("obj_0000" + o[5:]))
            elif o[:6] == "cuboid":
                tmp = o.split("_")
                obj.append(Cuboid([int(tmp[1]), int(tmp[2]), int(tmp[3])]))
        return obj

    @staticmethod
    def create_furniture(fur_id, fur_poses):
        fur = []
        for f in fur_id:
            if f == "table":
                fur.append(Table())
            elif f == "shelf":
                fur.append(Shelf())
            elif f == "tunnel":
                fur.append(Tunnel())
        return fur
