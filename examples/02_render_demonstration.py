#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import time
import numpy as np
import pinocchio as pin
from guided_tamp_benchmark.models.robots import *
from guided_tamp_benchmark.models.furniture import *
from guided_tamp_benchmark.tasks.base_task import BaseTask
from guided_tamp_benchmark.tasks.shelf_task import ShelfTask
from guided_tamp_benchmark.tasks.renderer import Renderer


# todo: change to one of the existing demonstrations
class TestTask(BaseTask):

    def __init__(self):
        super().__init__('test', 0, PandaRobot(), 0)

task = TestTask()
# task = ShelfTask(0, PandaRobot(), 0)
# task.furniture.clear()
# f = pin.SE3(np.eye(4))
# f.translation[0] = 1.
# task.furniture.append(Table(pose=f.homogeneous, desk_size=[1., 1., 0.7]))
# task.furniture.append(Shelf(position=[-1, 0, 2], rpy=[0, 0, 0], display_inside_shelf=True))
# task.furniture.append(Tunnel(position=[0, 2, 0], rpy=[0, 0, 0], lengths=[1, 1, 1]))

r = Renderer(task=task)
r.animate_demonstration(fps=1)

time.sleep(5.)
