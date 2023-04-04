#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 17.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>


import os
from typing import List
import xml.etree.ElementTree as ElTree
from pinocchio import XYZQUATToSE3
import numpy as np

from guided_tamp_benchmark.models.objects import BaseObject
from guided_tamp_benchmark.models.utils import get_ycbv_data_directory


class ObjectYCBV(BaseObject):
    _all_handles = [
        "handleZpx",
        "handleZmx",
        "handleYmx",
        "handleYpx",
        "handleZpxSm",
        "handleZmxSm",
        "handleZpxSp",
        "handleZmxSp",
        "handleZpy",
        "handleZmy",
        "handleXmy",
        "handleXpy",
        "handleZpySm",
        "handleZmySm",
        "handleZpySp",
        "handleZmySp",
        "handleYpz",
        "handleYmz",
        "handleXmz",
        "handleXpz",
        "handleYpzSm",
        "handleYmzSm",
        "handleYpzSp",
        "handleYmzSp",
    ]

    def __init__(self, object_name: str):
        """
        creates an object with attached srdf and urdf files for HPP

        :param object_name: string with the objects name, example: "obj_000002" or
         "obj_000012"
        """
        super().__init__(create_srdf_file=False)
        self.name = object_name

        self.srdfFilename = str(
            get_ycbv_data_directory().joinpath("srdf/" + object_name + ".srdf")
        )
        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(
                self.urdf(
                    name=self.name,
                    obj_path=str(
                        get_ycbv_data_directory().joinpath(f"meshes/{self.name}.obj")
                    ),
                )
            )

    @classmethod
    def initial_configuration(cls) -> List[float]:
        """
        generates initial configuration of cuboid in [x, y, z, i, j, k, w]

        :return: initial configuration of cuboid considering its size
        """
        return [
            0.0,
            0.0,
            0.0,
        ] + [0, 0, 0, 1]

    def handles(self, prefix: str = ""):
        """
        This function returns list of all handles prepended with the prefix.
        Handle description following:
                    HandleAbc(Sd)
                        A - X/Y/Z from which coordinate will the gripper come from

                        b - m/p if the approach is from minus (m) or plus (p)

                        c - x/y/z is coordinate in which the width of the grip is
                        represented

                        S - optional S means that the handle is a side handle

                        d - m/p argument tells if the side handle is on minus or plus
                        side of cuboid on the third axis

        :param prefix: prefix for handle name
        :return: list of handles [prefix + handleAbc, prefix + handleAbc, ...]
        """

        if self.name == "obj_000002":
            ids = [i for i in range(8)]
        elif self.name == "obj_000003":
            ids = [i for i in range(8)]
        elif self.name == "obj_000004":
            ids = [i for i in range(4)] + [i for i in range(8, 12)]
        elif self.name == "obj_000005":
            ids = [8, 9, 10, 11]
        elif self.name == "obj_000012":
            ids = [8, 9, 10, 11]
        elif self.name == "obj_000021":
            ids = [i for i in range(24)]
        else:
            return False

        return [prefix + self._all_handles[i] for i in ids]

    def contact_surfaces(self, prefix: str = ""):
        """
        This function returns the list of all contact surface defined by the object.
        :param prefix: prefix for contact surface name
        :return: name as string prefix + "box_surface"
        """
        return prefix + f"{self.name}_surface"

    def get_handles_poses(self):
        """
        This function parses .srdf file and returns poses for all handles.
        :return: dictionary with handle names as keys and 4x4 arrays as values
        """
        tree = ElTree.parse(self.srdfFilename)
        handles = {}
        for child in tree.getroot():
            if child.tag == "handle":
                assert child[0].tag == "position"
                xyz_wxyz = [float(x) for x in child[0].text.split()]
                xyz_xyzw = xyz_wxyz[:3] + xyz_wxyz[4:] + [xyz_wxyz[3]]
                handles[child.attrib["name"]] = np.array(XYZQUATToSE3(xyz_xyzw))
        return handles

    @staticmethod
    def urdf(name: str, obj_path: str):
        """
        this function generates text for .urdf file with given parameters to create
        ycbv object

        :param name: name of the ycbv object to be created
        :return: text of .urdf file with the description of ycbv object
        """

        urdf = f""" 
        <robot name="{name}">
           <link name="base_link">
              <visual>
                 <geometry>
                    <mesh filename="{obj_path}" scale="0.001 0.001 0.001"/>
                 </geometry>
                 <material name="mat_part0">
                    <color rgba="1.0 1.0 1.0 1.0"/>
                 </material>
              </visual>
              <collision>
                 <geometry>
                    <mesh filename="{obj_path}" scale="0.001 0.001 0.001"/>
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
