#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 15.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

class Task:
    def __init__(self, demo, robot):
        self.demo = demo
        self.robot = robot
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
