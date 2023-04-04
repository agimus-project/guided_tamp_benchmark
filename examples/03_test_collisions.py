#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.02.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from guided_tamp_benchmark.models.robots import *
from guided_tamp_benchmark.tasks import *
from guided_tamp_benchmark.core import Configuration

import numpy as np
import dill

with open(f"paper_benchmark_results.pkl", 'rb') as r:
    bench = dill.load(r)

planners = ['hpp_shortcut', 'hpp', 'pddl', 'multi_contact', 'multi_contact_shortcut']
task = ['shelf', 'tunnel', 'waiter']
id = [0, 1, 2]
robot = ['panda', 'ur5', 'kmr_iiwa']
robot_pose = [i for i in range(11)]
seed = [i for i in range(0, 10)]

# insert your configurations here
configs = [Configuration.from_numpy(np.array(x), len(KukaMobileIIWARobot().initial_configuration())) for x in
           bench[planners[3]][task[2]][id[0]][robot[2]][robot_pose[0]][seed[0]]["configs"]]

task = WaiterTask(0, KukaMobileIIWARobot(), 1)
config = Configuration(task.robot.initial_configuration(), task.demo.objects_poses[:, 0])

# checks if the given configuration has contacts or not
print(task._check_place_constraint(configs[1500]))

# checks if the given configuration is in collision or not
print(task._check_config_for_collision(configs[5]))

# chcecks if the give path is in collision or not
path = None  # instert your path here
print(task._check_path_for_collision(path, delta=0.0001))

# checks if configuration has grasps or not
task._check_grasp_constraint(configs[0], delta=0.0001)
