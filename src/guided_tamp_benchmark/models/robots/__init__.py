#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from .base import BaseRobot
from .kuka_iiwa_robot import KukaIIWARobot
from .kuka_kmr_iiwa_robot import KukaMobileIIWARobot
from .panda_robot import PandaRobot
from .ur5_robot import UR5Robot
from pathlib import Path

def get_robot_models_path():
    return Path(__file__).absolute().parent
