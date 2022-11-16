#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from tasks.base_task import Task


class ShelfTask(Task):
    def __init__(self, demo=None, **kwargs):
        super().__init__(demo, **kwargs)
