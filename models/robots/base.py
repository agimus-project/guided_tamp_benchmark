#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from abc import abstractmethod
from typing import List


class BaseRobot(object):

    @abstractmethod
    def initial_configuration(self) -> List[float]:
        """Return initial configuration of the object."""
        pass

    @abstractmethod
    def reach_m(self) -> float:
        """Return maximum reach of robot end effector in meters."""
        pass

    @abstractmethod
    def get_actuated_joint_names(self) -> List[str]:
        """Return list of names of actuated joints"""
        pass

    @classmethod
    @abstractmethod
    def get_pyphysx_robot(cls):

        pass

    @classmethod
    @abstractmethod
    def get_gripper_name(cls) -> str:
        """return name of the gripper of the robot"""
        pass

    @abstractmethod
    def modify_open_gripper(self, config) -> List[float]:

        pass
