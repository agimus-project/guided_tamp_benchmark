#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import numpy as np
from typing import List

from guided_tamp_benchmark.models.robots import BaseRobot
from guided_tamp_benchmark.models.utils import get_robots_data_directory


class UR5Robot(BaseRobot):
    urdfFilename = str(get_robots_data_directory().joinpath("ur5/ur5.urdf"))
    srdfFilename = str(get_robots_data_directory().joinpath("ur5/ur5.srdf"))
    name = "ur5"
    robot_type = "fixed"

    def __init__(self):
        pass

    def initial_configuration(self) -> List[float]:
        """Return the initial configuration of the robot."""
        return [0, -np.pi * 3 / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 4, 0, 0]

    def reach_m(self):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return 0.8

    def get_gripper_name(self):
        return "ur5/gripper"

    def footprint_size(self) -> list[float, float]:
        """Return the size of robots base"""
        return [0.2, 0.2]
