#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.02.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from guided_tamp_benchmark.models.robots import *
from guided_tamp_benchmark.tasks import *
from guided_tamp_benchmark.tasks.configuration import Configuration

task = TunnelTask(0, PandaRobot(), 1)
config = Configuration(task.robot.initial_configuration(), task.demo.objects_poses[:, 0])

# checks if the given configuration is in collision or not
print(task._check_config_for_collision(config))

# chcecks if the give path is in collision or not
path = [Configuration(task.robot.initial_configuration(), task.demo.objects_poses[:, i]) for i in
        range(len(task.demo.objects_poses[0]))]
print(task._check_path_for_collision(path))
