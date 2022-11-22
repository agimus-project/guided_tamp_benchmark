#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
import numpy as np
from typing import List

import models
from models.robots.base import BaseRobot


class KukaIIWARobot(BaseRobot):
    urdfFilename = os.path.dirname(models.__file__) + "/data/robots/kuka_iiwa/kuka_iiwa.urdf"
    srdfFilename = os.path.dirname(models.__file__) + "/data/robots/kuka_iiwa/kuka_iiwa.srdf"
    urdfSuffix = ""
    srdfSuffix = ""
    name = "Kuka_iiwa_robot"

    def __init__(self):
        pass

    def initial_configuration(cls) -> List[float]:
        """ Return the initial configuration of the robot. """
        return [0, -np.pi / 4, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4, 0, 0]

    def reach_m(cls):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return 0.8

    def get_gripper_name(cls):
        return "iiwa/gripper"
