#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from abc import abstractmethod
from typing import List, Tuple
import numpy as np

import pinocchio

import xml.etree.ElementTree as ET



class BaseRobot(object):
    urdfFilename = ""
    srdfFilename = ""
    urdfSuffix = ""
    srdfSuffix = ""
    name = "robot"
    robot_type = ""  # either mobile or fixed

    @abstractmethod
    def initial_configuration(self) -> List[float]:
        """Return initial configuration of the object."""
        pass

    @abstractmethod
    def reach_m(self) -> float:
        """Return maximum reach of robot end effector in meters."""
        pass

    def get_actuated_joint_names(self) -> List[str]:
        """Return list of names of actuated joints"""
        model = pinocchio.buildModelFromUrdf(self.urdfFilename)
        return model.names.tolist()[1:]

    @abstractmethod
    def get_gripper_name(self) -> str:
        """return name of the gripper of the robot"""
        pass

    def get_contact_surfaces(self) -> List[str]:
        """Return contact surfaces of the robot"""
        return []

    def get_contacts_info(self) -> dict:
        """returns contacts in a dictionary of a form contacts["name"] =
        {"link": str, "shapes": np.array}"""
        tree = ET.parse(self.srdfFilename)
        root = tree.getroot()
        contacts, _, _ = parse_contacts_grippers_handles(
            root, contacts=True, grippers=False, handles=False
        )
        return contacts

    def get_grippers_info(self) -> dict:
        """returns grippers in a dictionary of a form grippers["name"] =
        {"link": str, "pose": list,
         "clearance"" float}"""
        tree = ET.parse(self.srdfFilename)
        root = tree.getroot()
        _, grippers, _ = self._parse_contacts_grippers_handles(
            root, contacts=False, grippers=True, handles=False
        )
        return grippers

    def footprint_size(self) -> list[float, float]:
        """Return the size of robots base"""
        return [0.0, 0.0]

    def footprint_pos(self) -> list[float, float]:
        """Return the position of robots base"""
        return [0.0, 0.0]

    def _parse_contacts_grippers_handles(
            self, root, contacts=False, grippers=False, handles=False
    ) -> Tuple[dict, dict, dict]:
        """parses info from root of elementary tree xml parser of .srdf file. Returns
        dictionary with the wanted info in following format.
        contacts["name"] = {"link": str, "shapes": np.array},
        handles["name"] = {"link": str, "pose": list, "clearance"" float},
        grippers["name"] = {"link": str, "pose": list, "clearance"" float}"""
        contact_dict, gripper_dict, handle_dict = {}, {}, {}
        for child in root:
            if child.tag == "contact" and contacts:
                try:
                    shapes = child[2].text.split()
                    points = child[1].text.split()
                    parsed_shapes = []
                    while 0 < len(shapes):
                        s = int(shapes.pop(0))
                        shape = []
                        while True:
                            n = int(shapes.pop(0))
                            shape.append(
                                [
                                    float(points[3 * n]),
                                    float(points[3 * n + 1]),
                                    float(points[3 * n + 2]),
                                ]
                            )
                            s -= 1
                            if s == 0:
                                break
                        parsed_shapes.append(shape)

                    contact_dict[child.attrib["name"]] = {
                        "link": child[0].attrib["name"],
                        "shapes": np.array(parsed_shapes),
                    }
                except Exception:
                    raise NameError("srdf file is missing handles!")
            if child.tag == "gripper" and grippers:
                try:
                    gripper_dict[child.attrib["name"]] = {
                        "link": child[1].attrib["name"],
                        "pose": [float(n) for n in child[0].text.split()],
                        "clearance": float(child.attrib["clearance"]),
                    }
                except Exception:
                    raise NameError("srdf file is missing grippers!")
            if child.tag == "handle" and handles:
                try:
                    handle_dict[child.attrib["name"]] = {
                        "link": child[1].attrib["name"],
                        "pose": [float(n) for n in child[0].text.split()],
                        "clearance": float(child.attrib["clearance"]),
                    }
                except Exception:
                    raise NameError("srdf file is mssing handles!")

        return contact_dict, gripper_dict, handle_dict


