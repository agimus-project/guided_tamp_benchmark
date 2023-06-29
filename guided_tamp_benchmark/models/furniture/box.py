#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 13.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
from typing import List, Union
import numpy as np
from pinocchio.rpy import matrixToRpy

from guided_tamp_benchmark.models.furniture.base import FurnitureObject


class Box(FurnitureObject):
    name = "box"

    def __init__(self, pose: np.array, box_size: Union[List[float], float]) -> None:
        """
        will generate .urdf and .srdf file for environmental object box
        This object can be passed to hpp function loadEnvironmentObject() as argument.
        :param pose: 4x4 pose matrix
        :param desk_size: size of the box in [x, y, z] or single float for symmetric
         box size
        :param leg_display: True if legs should be displayed, else table will appear as
        a box
        """

        super().__init__()
        if isinstance(box_size, float):
            box_size = [box_size] * 3
        assert len(box_size) == 3
        assert pose.shape == (4, 4)

        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(
                self.urdf(
                    size=box_size,
                    pos=pose[:3, 3],
                    rot=matrixToRpy(pose[:3, :3]),
                )
            )
        with os.fdopen(self.fd_srdf, "w") as f:
            f.write(self.srdf())

    @staticmethod
    def urdf(
        pos: List[float],
        rot: List[float],
        size: List[float],
        material: str = "brown",
        color_rgba: str = "0.43 0.34 0.24 0.9",
    ):
        """
        this function generates text for .urdf file with given parameters to create
        table object

        :param pos: position of the box in [x, y, z]
        :param rot: rotation of the box in [r, p, y]
        :param size: size of the box in [x, y, z]
        :param material: optional material name
        :param color_rgba:  optional color of table
        :return: text of .urdf file with the description of table object
        """

        return f"""<robot name="box">
<link name="base_link">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
</link>
<link name="box">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {0} {0}" rpy="0 0 0" />
        <geometry>
            <box size="{size[0]} {size[1]} {size[2]}"/>
        </geometry>
        <material name="{material}">
            <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {0} {0}" rpy="0 0 0" />
        <geometry>
            <box size="{size[0]} {size[1]} {size[2]}"/>
        </geometry>
    </collision>
</link>

<joint name="main_link" type="fixed">
    <origin rpy="{rot[0]} {rot[1]} {rot[2]}" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
    <parent link="base_link"/>
    <child link="box"/>
</joint>
</robot>
"""

    @staticmethod
    def srdf():
        """
        this function generates text for .srdf file with given parameters to create
        contact surfaces

        :return: text of .srdf file of box object
        """
        return """<?xml version="1.0"?>
                <robot name="box">
                </robot>"""
