#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 15.11.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>


from typing import List

from guided_tamp_benchmark.models.robots import (
    PandaRobot,
    UR5Robot,
    KukaMobileIIWARobot,
    KukaIIWARobot,
    BaseRobot,
)


def get_robot(robot_name: str) -> BaseRobot:
    """Returns a robot instance based on robot name"""
    if robot_name == "panda":
        return PandaRobot()
    elif robot_name == "ur5":
        return UR5Robot()
    elif robot_name == "kuka_iiwa":
        return KukaIIWARobot()
    elif robot_name == "kmr_iiwa":
        return KukaMobileIIWARobot()
    else:
        raise ValueError(f"Unknown robot '{robot_name}'")


class ExtendedRobot(BaseRobot):
    def __init__(self, robot_name: str):
        self.robot = get_robot(robot_name)
        self.urdfFilename = (
            self.robot.urdfFilename[
                : self.robot.urdfFilename.find(self.robot.name + ".urdf")
            ]
            + self.robot.name
            + "_extended.urdf"
        )

        self.srdfFilename = self.robot.srdfFilename
        self.name = "extended_" + self.robot.name
        self.robot_type = self.robot.robot_type

    def initial_configuration(self) -> List[float]:
        """Return the initial configuration of the robot."""
        return [0] + self.robot.initial_configuration()

    def reach_m(self):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return self.robot.reach_m()

    def get_gripper_name(self):
        return self.robot.get_gripper_name()

    def get_contact_surfaces(self):
        return self.robot.get_contact_surfaces()
