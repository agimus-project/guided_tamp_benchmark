#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 13.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
from typing import List, Union
import numpy as np
from pinocchio.rpy import matrixToRpy

from guided_tamp_benchmark.models.furniture.base import FurnitureObject


class Table(FurnitureObject):
    name = "table"

    def __init__(
        self, pose: np.array, desk_size: Union[List[float], float], leg_display=True
    ) -> None:
        """
        will generate .urdf and .srdf file for environmental object table
        This object can be passed to hpp function loadEnvironmentObject() as argument.
        :param pose: 4x4 pose matrix
        :param desk_size: size of the desk in [x, y, z] or single float for symmetric
         desk size
        :param leg_display: True if legs should be displayed, else table will appear as
        a box
        """

        super().__init__()
        if isinstance(desk_size, float):
            desk_size = [desk_size] * 3
        assert len(desk_size) == 3
        assert pose.shape == (4, 4)

        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(
                self.urdf(
                    size=desk_size,
                    legs=leg_display,
                    pos=pose[:3, 3],
                    rot=matrixToRpy(pose[:3, :3]),
                )
            )
        with os.fdopen(self.fd_srdf, "w") as f:
            f.write(self.srdf(size=desk_size))

    def contact_surfaces(self, prefix: str = ""):
        """
        This function returns the list of all contact surface defined by the object.

        :param prefix: prefix for contact surface name
        :return: name as a list of strings [prefix + "desk"]
        """

        return [prefix + "table_surface"]

    @staticmethod
    def urdf(
        pos: List[float],
        rot: List[float],
        size: List[float],
        legs=True,
        material: str = "brown",
        color_rgba: str = "0.43 0.34 0.24 0.9",
    ):
        """
        this function generates text for .urdf file with given parameters to create
        table object

        :param pos: position of the table in [x, y, z]
        :param rot: rotation of the table in [r, p, y]
        :param size: size of the desk in [x, y]
        :param legs: True if legs should be displayed, else table will appear as a box
        :param material: optional material name
        :param color_rgba:  optional color of table
        :return: text of .urdf file with the description of table object
        """

        if legs:
            return f"""<robot name="table">
<link name="base_link">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
</link>
<link name="desk">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {0} {-0.015}" rpy="0 0 0" />
        <geometry>
            <box size="{size[0]} {size[1]} {0.03}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {0} {-0.015}" rpy="0 0 0" />
        <geometry>
            <box size="{size[0]} {size[1]} {0.03}"/>
        </geometry>
    </collision>
</link>
<link name="leg1">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{size[0] / 2 - 0.025} {size[1] / 2 - 0.025} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{0.05} {0.05} {size[2]}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{size[0] / 2 - 0.025} {size[1] / 2 - 0.025} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{0.05} {0.05} {size[2]}"/>
        </geometry>
    </collision>
</link>
<link name="leg2">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{-size[0] / 2 + 0.025} {size[1] / 2 - 0.025} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{0.05} {0.05} {size[2]}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{-size[0] / 2 + 0.025} {size[1] / 2 - 0.025} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{0.05} {0.05} {size[2]}"/>
        </geometry>
    </collision>
</link>
<link name="leg3">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{-size[0] / 2 + 0.025} {-size[1] / 2 + 0.025} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{0.05} {0.05} {size[2]}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{-size[0] / 2 + 0.025} {-size[1] / 2 + 0.025} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{0.05} {0.05} {size[2]}"/>
        </geometry>
    </collision>
</link>
<link name="leg4">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{size[0] / 2 - 0.025} {-size[1] / 2 + 0.025} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{0.05} {0.05} {size[2]}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{size[0] / 2 - 0.025} {-size[1] / 2 + 0.025} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{0.05} {0.05} {size[2]}"/>
        </geometry>
    </collision>
</link>

<joint name="main_link" type="fixed">
    <origin rpy="{rot[0]} {rot[1]} {rot[2]}" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
    <parent link="base_link"/>
    <child link="desk"/>
</joint>
<joint name="leg1_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="desk"/>
    <child link="leg1"/>
</joint>
<joint name="leg2_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="desk"/>
    <child link="leg2"/>
</joint>
<joint name="leg3_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="desk"/>
    <child link="leg3"/>
</joint>
<joint name="leg4_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="desk"/>
    <child link="leg4"/>
</joint>
</robot>
"""  # noqa E501
        else:
            return f"""<robot name="table">
<link name="base_link">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
</link>
<link name="desk">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {0} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{size[0]} {size[1]} {size[2]}"/>
        </geometry>
        <material name="{material}">
            <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {0} {-size[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{size[0]} {size[1]} {size[2]}"/>
        </geometry>
    </collision>
</link>

<joint name="main_link" type="fixed">
    <origin rpy="{rot[0]} {rot[1]} {rot[2]}" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
    <parent link="base_link"/>
    <child link="desk"/>
</joint>
</robot>
"""

    @staticmethod
    def srdf(size: List[float]):
        """
        this function generates text for .srdf file with given parameters to create
        contact surfaces

        :param size: size of the desk in [x, y]
        :return: text of .srdf file with contact surfaces for the table object
        """
        return f"""<?xml version="1.0"?>
<robot name="table">
<contact name="table_surface">
    <link name="desk"/>
        <point>
        {size[0] / 2}   {size[1] / 2}  0        {size[0] / 2}    {-size[1] / 2} 0
        {-size[0] / 2}  {-size[1] / 2} 0        {-size[0] / 2}   {size[1] / 2}  0
        </point>
        <shape>4 3 2 1 0</shape>
</contact>
</robot>"""
