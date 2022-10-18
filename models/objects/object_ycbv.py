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
        self.fd_srdf, self.srdfFilename = tempfile.mkstemp(suffix=".srdf", text=True)

        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(self.urdf(name=self.name))
        with os.fdopen(self.fd_srdf, "w") as f:
            f.write(self.srdf())

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

    def initial_configuration(self) -> List[float]:
        """
        generates initial configuration of cuboid in [x, y, z, i, j, k, w]

        :return: initial configuration of cuboid considering its size
        """
        return [0.0, 0, self.lengths[2] / 2 + 0.001, ] + [0, 0, 0, 1]

    def handles(self, prefix: str = ""):
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

        return [prefix + self._all_handles[h] for h in self.active_handles()]


    def contact_surfaces(self, prefix: str = ""):
        """
        This function returns the list of all contact surface defined by the object.
        :param prefix: prefix for contact surface name
        :return: name as string prefix + "box_surface"
        """
        return prefix + f"{self.name}_surface"

    def __del__(self):
        os.unlink(self.urdfFilename)
        os.unlink(self.srdfFilename)

    @staticmethod
    def urdf(name: str):
        """
        this function generates text for .urdf file with given parameters to create ycbv object

        :param name: name of the ycbv object to be created
        :return: text of .urdf file with the description of ycbv object
        """

        urdf =  f""" 
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

    def srdf(self):
        pos = []
        clearance = []
        points = ""
        shape = ""
        if self.name == "obj_000002":
            pos = ["0, 0, 0.06, 0.7071068, 0, 0.7071068, 0", "0, 0, -0.07,  0.7071068, 0, -0.7071068, 0",
                   "0, 0.03, 0, 0.5, 0.5, -0.5, -0.5", "0, -0.03, 0, 0.5, -0.5, -0.5, 0.5",
                   "0, -0.05, 0.07, 0.65328148, 0.27059805, 0.65328148, 0.27059805",
                   "0, 0.05, 0.07, 0.65328148, -0.27059805, 0.65328148, -0.27059805",
                   "0, -0.05, -0.07, 0.65328148, -0.27059805, -0.65328148,  0.27059805",
                   "0, 0.05, -0.07, 0.65328148, 0.27059805, -0.65328148, -0.27059805"]
            clearance = [0.075] * 8
            points = "0.036  0.082  0.107   0.036  0.082 -0.107    0.036 -0.082  -0.107    0.036 -0.082  0.107 " \
                     "-0.036  0.082  0.107  -0.036  0.082 -0.107   -0.036 -0.082  -0.107   -0.036 -0.082  0.107"
            shape = "4 3 2 1 0     4 4 5 6 7     4 1 2 6 5     4 4 7 3 0     4 0 1 5 4     4 7 6 2 3"
        elif self.name == "obj_000003":
            pos = ["0, 0, 0.05, 0.71920114, 0.01363871, 0.69452447, -0.0141233",
                   "0, 0, -0.05, 0.71920114, 0.01363871, -0.69452447, 0.0141233",
                   "0, 0, 0, 0.5, 0.5, -0.5, -0.5", "0, 0, 0, 0.5, -0.5, -0.5, 0.5",
                   "0, -0.01, 0.05, 0.6592359, 0.28782689, 0.6470617, 0.25273478",
                   "0, 0.01, 0.05, 0.29158667, 0.65758156, -0.25081292, -0.64780907",
                   "0, -0.01, -0.05, 0.66967452, -0.26262584, -0.63625219, 0.27883124",
                   "0, 0.01, -0.05, -0.24630638, 0.64280778, -0.29174007, 0.66409265"]
            clearance = [0.05] * 4 + [0.06] * 4
            points = "0.021 0.048 0.09   0.028 0.044 -0.09   0.024 -0.05 -0.089   0.017 -0.046 0.089   " \
                     "-0.026 0.049 0.087   -0.018 0.046 -0.09   -0.022 -0.048 -0.09   -0.029 -0.044 0.09"
            shape = "4 3 2 1 0     4 4 5 6 7     4 1 2 6 5     4 4 7 3 0     4 0 1 5 4    4 7 6 2 3"
        elif self.name == "obj_000004":
            pos = ["0, 0, 0, 0.7071068, 0, 0.7071068, 0", "0, 0, 0, 0.7071068, 0, -0.7071068, 0",
                   "0, 0, 0, 0.5, 0.5, -0.5, -0.5", "0, 0, 0, 0.5, -0.5, -0.5, 0.5", "0, 0, 0, 0.5, -0.5, 0.5, 0.5",
                   "0, 0, 0, 0.5, 0.5, -0.5, 0.5", "0, 0, 0, 0.7071068, -0.7071068, 0, 0",
                   "0, 0, 0, 0, 0, -0.7071068, 0.7071068"]
            clearance = [0.07] * 8
            points = "0.035 0.035 0.052   0.035 -0.035 0.052   -0.035 -0.035 0.052   -0.035 0.035 0.052   " \
                     "0.035 0.035 -0.052   0.035 -0.035 -0.052   -0.035 -0.035 -0.052   -0.035 0.035 -0.052"
            shape = "4 3 2 1 0     4 4 5 6 7"
        elif self.name == "obj_000005":
            pos = ["0, 0, 0.05, 0.58964632, -0.39027839,  0.58964632,  0.39027839",
                   "0, 0, -0.05, 0.58964632,  0.39027839, -0.58964632,  0.39027839",
                   "0, 0, 0, 0.6929114, -0.6929114, 0.14097442, -0.14097442",
                   "0, 0, 0, 0.14097442, -0.14097442, -0.6929114,   0.6929114"]
            clearance = [0.065, 0.075] + [0.065] * 2
            points = "0.043 -0.0052 -0.096   0.032 -0.029 -0.096   -0.045 0.0059 -0.096   -0.033 0.030 -0.096"
            shape = "4 0 1 2 3"
        elif self.name == "obj_000012":
            pos = ["0, 0, 0.08, 0.5, -0.5, 0.5, 0.5", "0, 0, -0.08, 0.5, 0.5, -0.5, 0.5, ",
                   "0, 0, 0, 0.7071068, -0.7071068 0, 0", "0.03, 0, 0, 0, 0, -0.7071068, 0.7071068"]
            clearance = [0.075] * 2 + [0.06] * 2
            points = "0.048 0.032 -0.126   0.048 -0.032 -0.126   -0.048 -0.032 -0.126   -0.048 0.032 -0.126"
            shape = "4 0 1 2 3"
        elif self.name == "obj_000021":
            pos = [#"handleZpx", "handleZmx", "handleXmx", "handleXpx"
                "0, 0, 0, 0.7071068, 0, 0.7071068, 0", "0, 0, 0, 0.7071068, 0, -0.7071068, 0",
                "0, 0, 0, 0.5, -0.5, -0.5, 0.5", "0, 0, 0, 0.5, 0.5, -0.5, -0.5",

                # "handleZpxSm", "handleZmxSm", "handleZpxSp", "handleZmxSp"
                "0, -0.0125, 0, 0.65328148, 0.27059805, 0.65328148, 0.27059805",
                "0, -0.0125, 0, 0.65328148, -0.27059805, -0.65328148, 0.27059805",
                "0, 0.0125, 0, 0.65328148, -0.27059805, 0.65328148, -0.27059805",
                "0, 0.0125, 0,  0.65328148,  0.27059805, -0.65328148, -0.27059805",

                #"handleZpy", "handleZmy", "handleXmy", "handleXpy"
                "0, 0, 0, 0.5, -0.5, 0.5, 0.5", "0, 0, 0, 0.5, 0.5, -0.5, 0.5",
                "0, 0, 0, 0.7071068, -0.7071068, 0, 0", "0, 0, 0, 0, 0, -0.7071068, 0.7071068",

                # "handleZpySm", "handleZmySm", "handleZpySp", "handleZmySp"
                "0, 0, 0, 0.65328148, -0.65328148, 0.27059805, 0.27059805",
                "0, 0, 0, 0.65328148, 0.65328148, -0.27059805, 0.27059805",
                "0, 0, 0, 0.27059805, -0.27059805, 0.65328148, 0.65328148",
                "0, 0, 0, 0.27059805, 0.27059805, -0.65328148, 0.65328148",

                # "handleYpz", "handleYmz", "handleXmz", "handleXpz"
                "0, 0, 0, 0.7071068, 0, 0, -0.7071068", "0, 0, 0, 0, 0.7071068, 0.7071068, 0",
                "0, 0, 0, 0, 1, 0, 0", "0, 0, 0, 0, 0, 1, 0",

                #"handleYpzSm", "handleYmzSm", "handleYpzSp", "handleYmzSp"
                "0, -0.0125, 0, 0, 0.9238795, -0.3826834, 0", "0, -0.0125, 0, 0, 0.9238795, 0.3826834, 0",
                "0, 0.0125, 0, 0, 0.3826834, -0.9238795, 0", "0, 0.0125, 0, 0, 0.3826834, 0.9238795, 0"]
            clearance = [0.04] * 4 + [0.075] * 2 + [0.04] * 10 + [0.075] * 4 + [0.04] * 4
            points = "0.026 0.039 0.026   0.026 0.039 -0.026   0.026 -0.039 -0.026   0.026 -0.039 0.026 " \
                     "-0.026 0.039 0.026   -0.026 0.039 -0.026   -0.026 -0.039 -0.026   -0.026 -0.039  -0.026"
            shape = "4 3 2 1 0     4 4 5 6 7     4 1 2 6 5     4 4 7 3 0     4 0 1 5 4     4 7 6 2 3"

        srdf = f"""
<?xml version="1.0" ?>
<robot name="{self.name}">
            """
        for i, h in enumerate(self.active_handles()):
            srdf += f"""
    <handle name="{self._all_handles[h]}" clearance="{clearance[i]}">
        <position> {pos[i]} </position>
        <link name="base_link"/>
    </handle>
                    """
        srdf += f"""                    
    <contact name="{self.name}_surface">
        <link name="base_link"/>
        <point>
             {points}
        </point>
        <shape> {shape} </shape>
    </contact>
</robot>
            """
        return srdf
