import unittest

import numpy as np
from guided_tamp_benchmark.models import *
from guided_tamp_benchmark.models.furniture import *
from guided_tamp_benchmark.models.objects import *
from guided_tamp_benchmark.models.robots import *

from xml.etree.ElementTree import ParseError


class ParsesTestCase(unittest.TestCase):
    furniture = [Shelf, Table, Tunnel]
    fur_args = [
        [
            {"pose": np.eye(4), "display_inside_shelf": True},
            {"pose": np.eye(4), "display_inside_shelf": False},
        ],
        [
            {"pose": np.eye(4), "desk_size": 0.5, "leg_display": True},
            {"pose": np.eye(4), "desk_size": 0.5, "leg_display": False},
        ],
        [{"pose": np.eye(4), "lengths": [0.06] * 3}],
    ]
    objects = [Cuboid, ObjectYCBV, Tray]
    obj_args = [
        [{"lengths": 0.06}],
        [
            {"object_name": "obj_000002"},
            {"object_name": "obj_000003"},
            {"object_name": "obj_000004"},
            {"object_name": "obj_000005"},
            {"object_name": "obj_000012"},
            {"object_name": "obj_000021"},
        ],
        [{"tray_size": 0.5}],
    ]
    robots = [KukaIIWARobot, KukaMobileIIWARobot, UR5Robot, PandaRobot]

    def test_robots(self):
        """test .srdf parser on robots"""
        for r in self.robots:
            try:
                r().get_contacts_info()
                r().get_grippers_info()
            except ParseError:
                self.fail()

    def test_objects(self):
        """test .srdf parser on objects"""
        for i, o in enumerate(self.objects):
            for arg in self.obj_args[i]:
                try:
                    o(**arg).get_handles_info()
                    o(**arg).get_contacts_info()
                except ParseError:
                    self.fail()

    def test_furniture(self):
        """test .srdf parser on furniture"""
        for i, f in enumerate(self.furniture):
            for arg in self.fur_args[i]:
                try:
                    f(**arg).get_contacts_info()
                except ParseError:
                    self.fail()


if __name__ == "__main__":
    unittest.main()
