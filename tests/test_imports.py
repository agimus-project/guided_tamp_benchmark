#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-06-26
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import sys


class TestImports(unittest.TestCase):
    def unload_package(self):
        for key in list(sys.modules.keys()):
            if key.startswith("guided_tamp_benchmark"):
                del sys.modules[key]

    def test_import_shelf_task(self):
        self.unload_package()
        from guided_tamp_benchmark.tasks.shelf_task import ShelfTask


if __name__ == "__main__":
    unittest.main()
