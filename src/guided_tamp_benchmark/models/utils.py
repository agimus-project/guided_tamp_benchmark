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
    return Path(__file__).resolve().parent.joinpath('data')


def get_robots_data_directory() -> Path:
    """Get path to the data of robots inside the models module"""
    return get_models_data_directory().joinpath('robots')


def get_ycbv_data_directory() -> Path:
    """Get path to the YCBV dataset data inside the models module"""
    return get_models_data_directory().joinpath('ycbv')


def parser(root, contacts=False, grippers=False, handles=False) -> Tuple[dict, dict, dict]:
    contact, gripper, handle = {}, {}, {}
    for child in root:
        if child.tag == "contact" and contacts:
            shapes = child[2].text.split()
            points = child[1].text.split()
            parsed_shapes = []
            while 0 < len(shapes):
                s = int(shapes.pop(0))
                shape = []
                while True:
                    n = int(shapes.pop(0))
                    shape.append([points[3 * n], points[3 * n + 1], points[3 * n + 2]])
                    s -= 1
                    if s == 0:
                        break
                parsed_shapes.append(shape)

            contact[child.attrib["name"]] = {"link": child[0].attrib["name"], "shapes": np.array(parsed_shapes)}
        if child.tag == "gripper" and grippers:
            pass
        if child.tag == "handle" and handles:
            pass

    return contact, gripper, handle
