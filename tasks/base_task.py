#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 15.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from tasks.demonstration import Demonstration
from typing import Optional


class Task:
    def __init__(self):
        self.demo: Optional[Demonstration] = None
        self.robot: Optional[object] = None
        pass

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
