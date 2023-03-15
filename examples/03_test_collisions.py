#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.02.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from guided_tamp_benchmark.models.robots import *
from guided_tamp_benchmark.tasks import *
from guided_tamp_benchmark.core.configuration import Configuration

from guided_tamp_benchmark.tasks.collisions import Collision

import numpy as np
import dill

with open(f"configs_shelf_2_panda.pkl", 'rb') as f:
    configs = dill.load(f)

configs = [Configuration.from_numpy(np.array(x), 9) for x in configs]

pass

task = ShelfTask(1, PandaRobot(), 1)
config = Configuration(task.robot.initial_configuration(), task.demo.objects_poses[:, 0])

# checks if the given configuration is in collision or not
print(task._check_config_for_collision(config))

# chcecks if the give path is in collision or not
path = [Configuration(task.robot.initial_configuration(), task.demo.objects_poses[:, i]) for i in
        range(len(task.demo.objects_poses[0]))]
print(task._check_path_for_collision(path))

task._check_place_constraint(configs[110], delta=0.0001)

pass