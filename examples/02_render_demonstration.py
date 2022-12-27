#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import time
from guided_tamp_benchmark.models.robots import *
from guided_tamp_benchmark.tasks import *
from guided_tamp_benchmark.tasks.renderer import Renderer

# demo_id, robot, pose_id
task = TunnelTask(0, PandaRobot(), 0)
r = Renderer(task=task)
r.animate_demonstration(fps=1)

time.sleep(5.)
