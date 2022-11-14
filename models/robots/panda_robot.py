#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
import pinocchio
import numpy as np
from typing import List

import models
from models.robots.base import BaseRobot


class PandaRobot(BaseRobot):
    urdfFilename = os.path.dirname(models.__file__) + "/data/robots/franka_panda/panda.urdf"
    srdfFilename = os.path.dirname(models.__file__) + "/data/robots/franka_panda/panda.srdf"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self):
        pass

    @classmethod
    def initial_configuration(cls) -> List[float]:
        """ Return the initial configuration of the robot. """
        return [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4, 0., 0.]

    @classmethod
    def reach_m(cls):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return 0.8

    def get_actuated_joint_names(self) -> List[str]:
        """Return list of names of actuated joints"""
        model = pinocchio.buildModelFromUrdf(self.urdfFilename)
        return model.names.tolist()[1:]

    @classmethod
    def get_gripper_name(cls):
        return "panda/gripper"
