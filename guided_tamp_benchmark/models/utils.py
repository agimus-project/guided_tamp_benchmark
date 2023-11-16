#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pathlib import Path

from guided_tamp_benchmark.models.robots import (
    PandaRobot,
    UR5Robot,
    KukaIIWARobot,
    KukaMobileIIWARobot,
    BaseRobot
)



def get_models_data_directory() -> Path:
    """Get path to the data of the models."""
    return Path(__file__).resolve().parent.joinpath("data")


def get_robots_data_directory() -> Path:
    """Get path to the data of robots inside the models module"""
    return get_models_data_directory().joinpath("robots")


def get_ycbv_data_directory() -> Path:
    """Get path to the YCBV dataset data inside the models module"""
    return get_models_data_directory().joinpath("ycbv")


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
