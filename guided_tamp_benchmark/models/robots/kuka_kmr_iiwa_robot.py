#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import numpy as np
from typing import List

from guided_tamp_benchmark.models.robots.base import BaseRobot
from guided_tamp_benchmark.models.utils import get_robots_data_directory


class KukaMobileIIWARobot(BaseRobot):
    urdfFilename = str(
        get_robots_data_directory().joinpath("kuka_kmr_iiwa/kuka_kmr_iiwa.urdf")
    )
    srdfFilename = str(
        get_robots_data_directory().joinpath("kuka_kmr_iiwa/kuka_kmr_iiwa.srdf")
    )
    name = "kmr_iiwa"
    robot_type = "mobile"

    def __init__(self):
        pass

    def initial_configuration(self) -> List[float]:
        """Return the initial configuration of the robot."""
        return [
            0.2,
            1.5,
            -np.pi,
            0,
            -np.pi / 4,
            0,
            -np.pi / 2,
            0,
            np.pi / 2,
            np.pi / 4,
            0,
            0,
        ]

    def reach_m(self):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return 0.8

    def get_gripper_name(self):
        return f"{self.name}/gripper"

    def get_contact_surfaces(self):
        return [
            f"{self.name}/kmr_surface",
        ]
