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
        """
        extended robot class, will create an extended robot for the given robot name
        with additional joint for the extended planner
        """
        self.robot = get_robot(robot_name)
        self.urdfFilename = self.robot.urdfFilename.replace(".urdf", "_extended.urdf")
        self.srdfFilename = self.robot.srdfFilename
        self.name = "" + self.robot.name
        self.robot_type = self.robot.robot_type

    def initial_configuration(self) -> List[float]:
        """
         :return: initial configuration of the robot
         """
        return [0] + self.robot.initial_configuration()

    def reach_m(self):
        """
        :return: Maximum reach of robot end effector in meters.
        """
        return self.robot.reach_m()

    def get_gripper_name(self):
        """
        :return: gripper name as string
        """
        return self.robot.get_gripper_name()

    def get_contact_surfaces(self):
        """
        :return: robot contact surfaces
        """
        return self.robot.get_contact_surfaces()
