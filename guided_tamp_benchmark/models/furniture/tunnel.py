#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 20.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
from typing import List
import numpy as np
from pinocchio.rpy import matrixToRpy

from guided_tamp_benchmark.models.furniture.base import FurnitureObject


class Tunnel(FurnitureObject):
    name = "tunnel"
    disabled_collision_links_for_robot = ["right_sideB", "left_sideB", "top_sideB"]

    def __init__(
        self,
        pose: np.array,
        lengths: List[float],
        tunnel_walls_thickness: float = 0.16,
        collision_walls_thickness: float = 0.1,
        walls_display=False,
    ) -> None:
        """
        will generate .urdf and .srdf file for environmental object tunnel.
        This object can be passed to hpp function loadEnvironmentObject() as argument.
        :param pose: 4x4 pose matrix
        :param lengths: sizes of the tunnel object [width in x, width in y, width in z]
        where x is tunnel length, y is the width of the tunnel hole and z is the height
         of the tunnel
        :param tunnel_walls_thickness: Thickness of the tunnel walls
        :param collision_walls_thickness: thickness of the walls preventing the robot to
        put object around tunnel
        :param walls_display: True if the walls preventing objects to go around tunnel
        should be displayed
        """
        super().__init__()
        assert len(lengths) == 3
        assert pose.shape == (4, 4)

        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(
                self.urdf(
                    pos=pose[:3, 3],
                    rpy=matrixToRpy(pose[:3, :3]),
                    lengths=lengths,
                    thickness=tunnel_walls_thickness,
                    collision_thickness=collision_walls_thickness,
                    disp_walls=walls_display,
                )
            )
        with os.fdopen(self.fd_srdf, "w") as f:
            f.write(self.srdf())

    def contact_surfaces(self, prefix: str = ""):
        """
        This function returns the list of all contact surface defined by the object.

        :param prefix: prefix for contact surface name
        :return: name as list of strings:
        [prefix + "top_shelf_surface", prefix + "inside_shelf_surface"]
        """

        return []

    @staticmethod
    def urdf(
        pos: List[float],
        rpy: List[float],
        lengths: List[float],
        thickness: float,
        collision_thickness: float,
        disp_walls=False,
        material: str = "grey",
        color_rgba: str = "0.713 0.725 0.725 0.8",
    ):
        """
        this function generates text for .urdf file with given parameters to create
        tunnel object
        :param pos: position of the tunnel in [x, y, z]
        :param rpy: rotation of the tunnel in [r, p, y]
        :param lengths: sizes of the tunnel object [width in x, width in y, width in z]
        :param thickness: Thickness of the tunnel walls
        :param collision_thickness: thickness of the walls preventing the robot to put
        object around tunnel
        :param disp_walls: True if the walls preventing objects to go around tunnel
        should be displayed
        :param material: optional material name
        :param color_rgba: optional color of the tunnel
        :return: text of .urdf file with the description of the tunnel object
        """

        if not disp_walls:
            walls_alpha = 0.0
        else:
            walls_alpha = 0.4

        return f"""
        <robot name="tunnel">
<link name="base_link">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
</link>
<link name="left_side">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {lengths[1] / 2 + thickness / 2} {lengths[2] / 2}" rpy="0 0 0"/>
        <geometry>
            <box size="{lengths[0]} {thickness} {lengths[2]}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {lengths[1] / 2 + thickness / 2} {lengths[2] / 2}" rpy="0 0 0"/>
        <geometry>
            <box size="{lengths[0]} {thickness} {lengths[2]}"/>
        </geometry>
    </collision>
</link>
<link name="right_side">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {-lengths[1] / 2 - thickness / 2} {lengths[2] / 2}" rpy="0 0 0"/>
        <geometry>
            <box size="{lengths[0]} {thickness} {lengths[2]}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {-lengths[1] / 2 - thickness / 2} {lengths[2] / 2}" rpy="0 0 0"/>
        <geometry>
            <box size="{lengths[0]} {thickness} {lengths[2]}"/>
        </geometry>
    </collision>
</link>
<link name="top_side">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz=" {0} {0} {lengths[2] / 2 + thickness / 2 + lengths[2] / 2}" rpy="0 0 0"/>
        <geometry>
            <box size="{lengths[0]} {lengths[1] + thickness * 2} {thickness}"/>
        </geometry>
        <material name="{material}">
        <color rgba="{color_rgba}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {0} {lengths[2] / 2 + thickness / 2 + lengths[2] / 2}" rpy="0 0 0"/>
        <geometry>
            <box size="{lengths[0]} {lengths[1] + thickness * 2} {thickness}"/>
        </geometry>
    </collision>
</link>
<link name="left_sideB">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {lengths[1] / 2 + 2.5} {lengths[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{collision_thickness} 5 5"/>
        </geometry>
        <material name="red">
        <color rgba="1 0 0 {walls_alpha}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {lengths[1] / 2 + 2.5} {lengths[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{collision_thickness} 5 5"/>
        </geometry>
    </collision>
</link>
<link name="right_sideB">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {-lengths[1] / 2 - 2.5} {lengths[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{collision_thickness} 5 5"/>
        </geometry>
        <material name="red">
        <color rgba="1 0 0 {walls_alpha}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {-lengths[1] / 2 - 2.5} {lengths[2] / 2}" rpy="0 0 0" />
        <geometry>
            <box size="{collision_thickness} 5 5"/>
        </geometry>
    </collision>
</link>
<link name="top_sideB">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
        <mass value="1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
        <origin xyz="{0} {0} {lengths[2] / 2 + (2.5 - lengths[2]) / 2 + lengths[2] / 2}" rpy="0 0 0"/>
        <geometry>
            <box size="{collision_thickness} {lengths[1]} {2.5 - lengths[2]}"/>
        </geometry>
        <material name="red">
        <color rgba="1 0 0 {walls_alpha}"/>
        </material>
    </visual>
    <collision>
        <origin xyz="{0} {0} {lengths[2] / 2 + (2.5 - lengths[2]) / 2 + lengths[2] / 2}" rpy="0 0 0"/>
        <geometry>
            <box size="{collision_thickness} {lengths[1]} {2.5 - lengths[2]}"/>
        </geometry>
    </collision>
</link>
<joint name="main_link" type="fixed">
    <origin rpy="{rpy[0]} {rpy[1]} {rpy[2]}" xyz="{pos[0]} {pos[1]} {pos[2]}"/>
    <parent link="base_link"/>
    <child link="top_side"/>
</joint>
<joint name="left_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_side"/>
    <child link="left_side"/>
</joint>
<joint name="right_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_side"/>
    <child link="right_side"/>
</joint>
<joint name="top_connect" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_side"/>
    <child link="top_sideB"/>
</joint>
<joint name="left_connectB" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_sideB"/>
    <child link="left_sideB"/>
</joint>
<joint name="right_connectB" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="top_sideB"/>
    <child link="right_sideB"/>
</joint>
</robot>
        """  # noqa E501

    @staticmethod
    def srdf():
        """
        this function generates text for .srdf file
        :return: text of .srdf file with contact surfaces for the tunnel object
        """
        return """<?xml version="1.0"?>
            <robot name="tunnel">
            </robot>
                """
