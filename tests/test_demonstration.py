#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-06-30
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest

from guided_tamp_benchmark.tasks.demonstration import Demonstration


class TestDemonstration(unittest.TestCase):
    def test_demonstration_ids(self):
        self.assertTrue(0 in Demonstration.get_demonstration_ids("shelf"))
        self.assertTrue(1 in Demonstration.get_demonstration_ids("shelf"))
        self.assertTrue(2 in Demonstration.get_demonstration_ids("shelf"))
        self.assertEqual(len(Demonstration.get_demonstration_ids("shelf")), 3)
        self.assertTrue(0 in Demonstration.get_demonstration_ids("tunnel"))
        self.assertTrue(0 in Demonstration.get_demonstration_ids("waiter"))
        self.assertEqual(len(Demonstration.get_demonstration_ids("tunnel")), 1)
        self.assertEqual(len(Demonstration.get_demonstration_ids("waiter")), 1)

    def test_number_of_robot_poses(self):
        self.assertEqual(
            11, Demonstration.get_number_of_robot_poses("shelf", 0, "panda")
        )


if __name__ == "__main__":
    unittest.main()
