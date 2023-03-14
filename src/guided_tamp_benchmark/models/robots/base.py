#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from abc import abstractmethod
from typing import List

import pinocchio

import xml.etree.ElementTree as ET

from guided_tamp_benchmark.models import parser


class BaseRobot(object):
    urdfFilename = ''
    srdfFilename = ''
    urdfSuffix = ''
    srdfSuffix = ''
    name = 'robot'

    @abstractmethod
    def initial_configuration(self) -> List[float]:
        """Return initial configuration of the object."""
        pass

    @abstractmethod
    def reach_m(self) -> float:
        """Return maximum reach of robot end effector in meters."""
        pass

    def get_actuated_joint_names(self) -> List[str]:
        """Return list of names of actuated joints"""
        model = pinocchio.buildModelFromUrdf(self.urdfFilename)
        return model.names.tolist()[1:]

    @abstractmethod
    def get_gripper_name(self) -> str:
        """return name of the gripper of the robot"""
        pass

    def get_contact_surfaces(self) -> List[str]:
        """ Return contact surfaces of the robot"""
        return []

    def get_contacts(self) -> dict:
        """returns contacts in a dictionary of a form contacts["name"] = {"link": str, "shapes": np.array}"""
        tree = ET.parse(self.srdfFilename)
        root = tree.getroot()
        contacts, _, _ = parser(root, contacts=True, grippers=False, handles=False)
        return contacts

    def get_grippers(self) -> dict:
        """returns contacts in a dictionary of a form contacts["name"] = {"link": str, "shapes": np.array}"""
        tree = ET.parse(self.srdfFilename)
        root = tree.getroot()
        _, grippers, _ = parser(root, contacts=False, grippers=True, handles=False)
        return grippers
