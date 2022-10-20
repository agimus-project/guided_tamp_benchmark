#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 17.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>


import os
import tempfile
from typing import List
from .base import BaseObject


class ObjectYCBV(BaseObject):
    rootJointType = "freeflyer"
    urdfSuffix = ""
    srdfSuffix = ""

    _all_handles = ["handleZpx", "handleZmx", "handleYmx", "handleYpx", "handleZpxSm", "handleZmxSm", "handleZpxSp",
                    "handleZmxSp", "handleZpy", "handleZmy", "handleXmy", "handleXpy", "handleZpySm", "handleZmySm",
                    "handleZpySp", "handleZmySp", "handleYpz", "handleYmz", "handleXmz", "handleXpz", "handleYpzSm",
                    "handleYmzSm", "handleYpzSp", "handleYmzSp"]

    def __init__(self, object_name: str):
        """
        creates an object with attached srdf and urdf files for HPP

        :param object_name: string whit the objecs name, example: "obj_000002" or "obj_000012"
        """
        super().__init__()
        self.name = object_name

        self.fd_urdf, self.urdfFilename = tempfile.mkstemp(suffix=".urdf", text=True)
        self.srdfFilename = "package://ycbv/srdf/" + object_name + ".srdf"

        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(self.urdf(name=self.name))

    def active_handles(self):
        """
        function that gives correct handle IDs for the current ycbv object

        :return: handle IDs in a list of int ex.: [1,2,3,4]
        """

        if self.name == "obj_000002":
            return [i for i in range(8)]
        elif self.name == "obj_000003":
            return [i for i in range(8)]
        elif self.name == "obj_000004":
            return [i for i in range(4)] + [i for i in range(8, 12)]
        elif self.name == "obj_000005":
            return [8, 9, 10, 11]
        elif self.name == "obj_000012":
            return [8, 9, 10, 11]
        elif self.name == "obj_000021":
            return [i for i in range(24)]

    @classmethod
    def initial_configuration(cls) -> List[float]:
        """
        generates initial configuration of cuboid in [x, y, z, i, j, k, w]

        :return: initial configuration of cuboid considering its size
        """
        return [0.0, 0.0, 0.0, ] + [0, 0, 0, 1]

    @classmethod
    def handles(cls, prefix: str = ""):
        """
        This function returns list of all handles prepended with the prefix.
        Handle description following:
                    HandleAbc(Sd)
                        A - X/Y/Z from which coordinate will the gripper come from

                        b - m/p if the approach is from minus (m) or plus (p)

                        c - x/y/z is coordinate in which the width of the grip is represented

                        S - optional S means that the handle is a side handle

                        d - m/p argument tells if the side handle is on minus or plus side of cuboid on the third axis

        :param prefix: prefix for handle name
        :return: list of handles [prefix + handleAbc, prefix + handleAbc, ...]
        """

        return [prefix + h for h in cls._all_handles]

    def contact_surfaces(self, prefix: str = ""):
        """
        This function returns the list of all contact surface defined by the object.
        :param prefix: prefix for contact surface name
        :return: name as string prefix + "box_surface"
        """
        return prefix + f"{self.name}_surface"

    def __del__(self):
        os.unlink(self.urdfFilename)

    @staticmethod
    def urdf(name: str):
        """
        this function generates text for .urdf file with given parameters to create ycbv object

        :param name: name of the ycbv object to be created
        :return: text of .urdf file with the description of ycbv object
        """

        urdf = f""" 
        <robot name="{name}">
           <link name="base_link">
              <visual>
                 <geometry>
                    <mesh filename="package://ycbv/meshes/{name}.obj" scale="0.001 0.001 0.001"/>
                 </geometry>
                 <material name="mat_part0">
                    <color rgba="1.0 1.0 1.0 1.0"/>
                 </material>
              </visual>
              <collision>
                 <geometry>
                    <mesh filename="package://ycbv/meshes/{name}.obj" scale="0.001 0.001 0.001"/>
                 </geometry>
              </collision>
              <inertial>
                 <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
                 <mass value="0.1"/>
                 <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
              </inertial>
           </link>
        </robot>
            """
        return urdf
