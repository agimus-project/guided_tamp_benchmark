#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from guided_tamp_benchmark.tasks import BaseTask


class ShelfTask(BaseTask):
    def __init__(self, robot=None):
        super().__init__("shelf", robot)
