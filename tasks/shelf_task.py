#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from tasks.base_task import BaseTask



class ShelfTask(BaseTask):
    def __init__(self, robot=None, **kwargs):
        super().__init__("shelf", robot, **kwargs)
