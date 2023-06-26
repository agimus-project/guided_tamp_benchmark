#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-06-26
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest


class TestImports(unittest.TestCase):
    def test_import_shelf_task_without_base_planner(self):
        # from guided_tamp_benchmark.core import BasePlanner
        from guided_tamp_benchmark.tasks.shelf_task import ShelfTask



if __name__ == '__main__':
    unittest.main()
