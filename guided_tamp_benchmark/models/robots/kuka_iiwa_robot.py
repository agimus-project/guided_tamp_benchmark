#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import numpy as np
from typing import List

from guided_tamp_benchmark.models.robots import BaseRobot
from guided_tamp_benchmark.models.utils import get_robots_data_directory


class KukaIIWARobot(BaseRobot):
    urdfFilename = str(get_robots_data_directory().joinpath("kuka_iiwa/kuka_iiwa.urdf"))
    srdfFilename = str(get_robots_data_directory().joinpath("kuka_iiwa/kuka_iiwa.srdf"))
    name = "kuka_iiwa"
    robot_type = "fixed"

    def initial_configuration(self) -> List[float]:
        """Return the initial configuration of the robot."""
        return [0, -np.pi / 4, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4, 0, 0]

    def reach_m(self):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return 0.8

    def get_gripper_name(self):
        return "iiwa/gripper"

    def footprint_size(self) -> list[float, float]:
        """Return the size of robots base"""
        return [0.2, 0.2]

    def footprint_pos(self) -> list[float, float]:
        """Return the position of robots base"""
        return [0.0, 0.05]
