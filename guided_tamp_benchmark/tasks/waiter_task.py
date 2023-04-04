#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 29.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from guided_tamp_benchmark.tasks import BaseTask
from guided_tamp_benchmark.models.robots import KukaMobileIIWARobot


class WaiterTask(BaseTask):
    def __init__(self, demo_id: int, robot: KukaMobileIIWARobot, robot_pose_id: int):
        super().__init__(
            "waiter", demo_id=demo_id, robot=robot, robot_pose_id=robot_pose_id
        )
