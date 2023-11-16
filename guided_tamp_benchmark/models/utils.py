#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pathlib import Path

from typing import Tuple
import numpy as np


def get_models_data_directory() -> Path:
    """Get path to the data of the models."""
    return Path(__file__).resolve().parent.joinpath("data")


def get_robots_data_directory() -> Path:
    """Get path to the data of robots inside the models module"""
    return get_models_data_directory().joinpath("robots")


def get_ycbv_data_directory() -> Path:
    """Get path to the YCBV dataset data inside the models module"""
    return get_models_data_directory().joinpath("ycbv")


def parse_contacts_grippers_handles(
    root, contacts=False, grippers=False, handles=False
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
