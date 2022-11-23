#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import time
from models.robots import *
from models.furniture import *
from tasks.base_task import BaseTask
from tasks.renderer import Renderer


# todo: change to one of the existing demonstrations
class TestTask(BaseTask):

    def __init__(self):
        super().__init__('test', 0, PandaRobot(), 0)


task = TestTask()
task.furniture.clear()
task.furniture.append(Table(position=[1, 0, 0.7], rpy=[0, 0, 0], desk_size=[1., 1., 0.7]))
# task.furniture.append(Shelf(position=[-1, 0, 2], rpy=[0, 0, 0], display_inside_shelf=True))
# task.furniture.append(Tunnel(position=[0, 2, 0], rpy=[0, 0, 0], lengths=[1, 1, 1]))

r = Renderer(task=task)
r.animate_demonstration(fps=1)

time.sleep(5.)
