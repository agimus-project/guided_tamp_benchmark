#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
import numpy as np
from typing import List

from models.robots.base import BaseRobot


# todo: discuss: Previous class parent was cass Robot from hpp.corbaserver.manipulation.robot
class KukaIIWARobot(BaseRobot):
    urdfFilename = os.path.dirname(__file__) + "data/kuka_iiwa/kuka_iiwa.urdf"
    srdfFilename = os.path.dirname(__file__) + "data/kuka_iiwa/kuka_iiwa.srdf"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self):
        pass

    @classmethod
    def initial_configuration(cls) -> List[float]:
        """ Return the initial configuration of the robot. """
        return [0, -np.pi / 4, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4, 0, 0]

    @classmethod
    def reach_m(cls):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return 0.8

    # todo: Will we need this function? Dependent on robot class from corbaserver.manipulation
    def get_actuated_joint_names(self) -> List[str]:
        """Return list of names of actuated joints"""
        pass

    # todo: Get new implementation with pyphysx
    @classmethod
    def get_pyphysx_robot(cls):
        pass

    @classmethod
    def get_gripper_name(cls):
        return "iiwa/gripper"

    def modify_open_gripper(self, config):
        ndof = len(self.initial_configuration())
        config[ndof - 2:ndof] = [0.039, 0.039]
        return config
