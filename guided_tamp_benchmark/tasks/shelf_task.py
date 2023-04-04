#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from guided_tamp_benchmark.tasks import BaseTask
from guided_tamp_benchmark.models.robots import BaseRobot


class ShelfTask(BaseTask):
    def __init__(self, demo_id: int, robot: BaseRobot, robot_pose_id: int):
        super().__init__(
            "shelf", demo_id=demo_id, robot=robot, robot_pose_id=robot_pose_id
        )
