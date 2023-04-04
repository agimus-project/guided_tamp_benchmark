#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 04.04.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>


class BasePlanner:
    name = None

    def solve(self):
        pass

    def path_len(self) -> float:
        pass
