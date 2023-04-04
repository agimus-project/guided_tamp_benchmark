#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 17.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
from typing import List
import numpy as np
from pinocchio.rpy import matrixToRpy

from guided_tamp_benchmark.models.furniture.base import FurnitureObject


class Shelf(FurnitureObject):
    name = "shelf"

    def __init__(self, pose: np.array, display_inside_shelf=True) -> None:
        """
        will generate .urdf and .srdf file for environmental object shelf.
        This object can be passed to hpp function loadEnvironmentObject() as argument.

        :param pose: 4x4 pose matrix
        :param display_inside_shelf:  True if inside of shelf should be displayed, else
        most of the shelf will be box
        """
        super().__init__()
        assert pose.shape == (4, 4)
        self.display_inside_shelf = display_inside_shelf

        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(
                self.urdf(
                    pos=pose[:3, 3],
                    rot=matrixToRpy(pose[:3, :3]),
                    inside=display_inside_shelf,
                )
            )
        with os.fdopen(self.fd_srdf, "w") as f:
            f.write(self.srdf(inside_shelf=display_inside_shelf))

    def contact_surfaces(self, prefix: str = ""):
        """
        This function returns the list of all contact surface defined by the object.

        :param prefix: prefix for contact surface name
        :return: name as list of strings:
        [prefix + "top_shelf_surface", prefix + "inside_shelf_surface"]
        """
        contact_surfaces = [
            prefix + "top_shelf_surface",
        ]
        if self.display_inside_shelf:
            contact_surfaces += [prefix + "inside_shelf_surface"]
        return contact_surfaces

    @staticmethod
    def urdf(
        pos: List[float],
        rot: List[float],
        inside=True,
        material: str = "brown",
        color_rgba: str = "0.43 0.34 0.24 0.9",
    ):
        """
        this function generates text for .urdf file with given parameters to create
         shelf object

        :param pos: position of the shelf in [x, y, z]
        :param rot: rotation of the shelf in [r, p, y]
        :param inside: True if inside of shelf should be displayed, else most of the
         shelf will be box
        :param material: optional material name
        :param color_rgba: optional color of shelf
        :return: text of .urdf file with the description of the shelf object
        """

        if inside:
            return f"""<robot name="shelf">
<link name="base_link">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
</link>
<link name="top_desk">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{-0.01} {0} {-0.0125}" rpy="0 0 0" />
        <geometry>
            <box size="{0.40} {1.20} {0.025}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
    <origin xyz="{-0.01} {0} {-0.0125}" rpy="0 0 0" />
        <geometry>
            <box size="{0.40} {1.20} {0.025}"/>
        </geometry>
    </collision>
</link>
<link name="box">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {0} {-0.8275}" rpy="0 0 0" />
        <geometry>
            <box size="{0.38} {1.20} {0.945}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {0} {-0.8275}" rpy="0 0 0" />
        <geometry>
            <box size="{0.38} {1.20} {0.945}"/>
        </geometry>
    </collision>
</link>
<link name="wall_left">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{-0.0075} {-0.59} {-0.19}" rpy="0 0 0" />
        <geometry>
            <box size="{0.365} {0.02} {0.33}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{-0.0075} {-0.59} {-0.19}" rpy="0 0 0" />
        <geometry>
            <box size="{0.365} {0.02} {0.33}"/>
        </geometry>
    </collision>
</link>
<link name="wall_center">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{-0.0075} {0.0} {-0.19}" rpy="0 0 0" />
        <geometry>
            <box size="{0.365} {0.02} {0.33}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{-0.0075} {0.0} {-0.19}" rpy="0 0 0" />
        <geometry>
            <box size="{0.365} {0.02} {0.33}"/>
        </geometry>
    </collision>
</link>
<link name="wall_right">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{-0.0075} {0.59} {-0.19}" rpy="0 0 0" />
        <geometry>
            <box size="{0.365} {0.02} {0.33}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{-0.0075} {0.59} {-0.19}" rpy="0 0 0" />
        <geometry>
            <box size="{0.365} {0.02} {0.33}"/>
        </geometry>
    </collision>
</link>
<link name="wall_back">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0.1825} {0} {-0.19}" rpy="0 0 0" />
        <geometry>
            <box size="{0.015} {1.20} {0.33}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0.1825} {0} {-0.19}" rpy="0 0 0" />
        <geometry>
            <box size="{0.015} {1.20} {0.33}"/>
        </geometry>
    </collision>
</link>

<joint name="main_link" type="fixed">
    <origin rpy="{rot[0]} {rot[1]} {rot[2]}" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
    <parent link="base_link"/>
    <child link="top_desk"/>
</joint>
<joint name="box_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_desk"/>
    <child link="box"/>
</joint>
<joint name="wall_L_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_desk"/>
    <child link="wall_left"/>
</joint>
<joint name="wall_C_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_desk"/>
    <child link="wall_center"/>
</joint>
<joint name="wall_R_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_desk"/>
    <child link="wall_right"/>
</joint>
<joint name="wall_B_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_desk"/>
    <child link="wall_back"/>
</joint>
</robot>
"""
        else:
            return f"""<robot name="shelf">
<link name="base_link">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
</link>
<link name="top_desk">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz=" {-0.01} {0} {-0.0125}" rpy="0 0 0" />
        <geometry>
            <box size="{0.40} {1.20} {0.025}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <origin xyz=" {-0.01} {0} {-0.0125}" rpy="0 0 0" />
    <geometry>
        <box size="{0.40} {1.20} {0.025}"/>
    </geometry>
</link>
<link name="box">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {0} {-0.6625}" rpy="0 0 0" />
        <geometry>
            <box size="{0.38} {1.20} {1.28}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {0} {-0.6625}" rpy="0 0 0" />
        <geometry>
            <box size="{0.38} {1.20} {1.28}"/>
        </geometry>
    </collision>
</link>


<joint name="main_link" type="fixed">
    <origin rpy="{rot[0]} {rot[1]} {rot[2]}" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
    <parent link="base_link"/>
    <child link="top_desk"/>
</joint>
<joint name="box_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_desk"/>
    <child link="box"/>
</joint>
</robot>
"""

    @staticmethod
    def srdf(inside_shelf=True):
        """
        this function generates text for .srdf file with given parameters to create
        contact surfaces

        :param inside_shelf: True if inside of shelf should be displayed, else most of
        the shelf will be box
        :return: text of .srdf file with contact surfaces for the shelf object
        """

        if inside_shelf:
            return """<?xml version="1.0"?>
                <robot name="shelf">
                <contact name="top_shelf_surface">
                        <link name="top_desk"/>
                           <point>
                              0.19 0.60 0   0.19 -0.60 0    -0.21 -0.60 0   -0.21 0.60 0
                          </point>
                        <shape>4 3 2 1 0</shape>
                    </contact>
                <contact name="inside_shelf_surface">
                        <link name="top_desk"/>
                         <point>
                             0.175 0.6 -0.355   0.175 0.01 -0.355   -0.19 0.01 -0.355
                            -0.19  0.60 -0.355  0.175 -0.6 -0.355   0.175 -0.01 -0.355
                            -0.19 -0.01 -0.355  -0.19 -0.60 -0.355
                         </point>
                         <shape>4 3 2 1 0     4 4 5 6 7</shape>
                      </contact>
                    </robot>
                """
        else:
            return """<?xml version="1.0"?>
                <robot name="shelf">
                <contact name="top_shelf_surface">
                        <link name="top_desk"/>
                           <point>
                          0.19 0.6 0    0.19 -0.6 0 -0.21 -0.6 0    -0.21 0.6 0
                          </point>
                        <shape>4 3 2 1 0</shape>
                    </contact>
                </robot>
                """
