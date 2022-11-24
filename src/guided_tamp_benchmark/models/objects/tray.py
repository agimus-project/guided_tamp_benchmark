#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 24.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
from typing import List, Union

from guided_tamp_benchmark.models.furniture.base import FurnitureObject


class Tray(FurnitureObject):

    def __init__(self, tray_size: Union[List[float], float],
                 leg_display=True) -> None:
        """
        will generate .urdf and .srdf file for object tray.
        :param desk_size: size of the tray in [x, y, z] or single float for symmetric tray
        """

        super().__init__()
        if isinstance(tray_size, float):
            tray_size = [tray_size] * 3
        assert len(tray_size) == 3

        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(self.urdf(size=tray_size))
        with os.fdopen(self.fd_srdf, "w") as f:
            f.write(self.srdf(size=tray_size))

    @classmethod
    def contact_surfaces(cls, prefix: str = ""):
        """
        This function returns the list of all contact surface defined by the object.

        :param prefix: prefix for contact surface name
        :return: name as a list of strings [prefix + "desk"]
        """

        return [prefix + "tray_surface"]

    @staticmethod
    def urdf(size: List[float], material: str = 'brown', color_rgba: str = '0.43 0.34 0.24 0.9'):
        """
        :param size: size of the tray in [x, y, z]
        :param material: optional material name
        :param color_rgba:  optional color of tray
        :return: text of .urdf file with the description of tray object
        """

        return f"""
            <robot name="tray">
                <link name="box">
                    <inertial>
                        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
                        <mass value="1"/>
                        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
                    </inertial>
                    <visual>
                        <origin xyz="0 0 0" rpy="0 0 0" />
                        <geometry>
                            <box size="{size[0]} {size[1]} {size[2]}"/>
                        </geometry>
                        <material name="{material}">
                        <color rgba="{color_rgba}"/>
                        </material>
                    </visual>
                    <collision>
                        <origin xyz="0 0 0" rpy="0 0 0" />
                        <geometry>
                            <box size="{size[0]} {size[1]} {size[2]}"/>
                        </geometry>
                    </collision>
                </link>
            </robot>
            """

    @staticmethod
    def srdf(size: List[float]):
        """
        this function generates text for .srdf file with given parameters to create contact surfaces

        :param size: size of the tray in [x, y, z]
        :return: text of .srdf file with contact surfaces for the tray object
        """
        return f"""
            <?xml version="1.0"?>
            <robot name="tray">
                <contact name="tray_surface">
                    <link name="box"/>
                        <point>
                        {size[0] / 2}   {size[1] / 2}  {size[1] / 2}       {size[0] / 2}    {-size[1] / 2} {size[1] / 2}
                        {-size[0] / 2}  {-size[1] / 2} {size[1] / 2}       {-size[0] / 2}   {size[1] / 2}  {size[1] / 2}
                        </point>
                        <shape>4 3 2 1 0</shape>
                </contact>
            </robot>
            """
