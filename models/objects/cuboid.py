#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 11.10.2022
#     Author: David Kovar <kovarda8@fel.cvut.cz>
#

import os
import tempfile
from typing import List


class Cuboid(object):
    rootJointType = "freeflyer"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, lengths, coords_display=False, side_handles=False) -> None:
        """
        Create urdf/srdf files for the box of given size.
        This object can be passed to hpp function loadEnvironmentObject() or loadObjectModel() as argument.

        :param lengths: sizes of the cuboid object [width in x, width in y, width in z]
        :param coords_display: will display coordinate system of the cuboid
        :param side_handles: If true, additional handles which are placed at the sides of the cuboid will be added
        """
        super().__init__()
        self.coords_disp = coords_display
        self.h1_active = False
        self.h2_active = False
        self.h3_active = False
        self.side_handles = side_handles
        if isinstance(lengths, float):
            lengths = [lengths] * 3
        else:
            pass

        assert len(lengths) == 3

        self.fd_urdf, self.urdfFilename = tempfile.mkstemp(suffix=".urdf", text=True)
        self.fd_srdf, self.srdfFilename = tempfile.mkstemp(suffix=".srdf", text=True)

        with os.fdopen(self.fd_urdf, "w") as f:
            f.write(self.urdf(lengths=lengths, coords_disp=coords_display))
        with os.fdopen(self.fd_srdf, "w") as f:
            f.write(self.srdf(lengths=lengths, side_handles=side_handles))

        self.lengths = lengths

    def initial_configuration(self) -> List[float]:
        """
        generates initial configuration of cuboid in [x, y, z, i, j, k, w]

        :return: initial configuration of cuboid considering its size
        """
        return [0.0, 0, self.lengths[2] / 2 + 0.001, ] + [0, 0, 0, 1]

    @staticmethod
    def urdf(lengths: List[float], coords_disp, material: str = 'red', color_rgba: str = '1 0.2 0.2 0.8') -> str:
        """
        this function generatestext for .urdf file with given parameters to create cuboid object

        :param lengths: sizes of the cuboid object [width in x, width in y, width in z]
        :param coords_disp: will add to the text of .urdf file additional part whith coordinate system of the cuboid
        :param material: optional material name
        :param color_rgba: optional color of cuboid
        :return: text of .urdf file with the description of cuboid object
        """

        base = f"""
            <robot name="box">

                <link name="base_link">
                    <inertial>
                        <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
                        <mass value="1"/>
                        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001" />
                    </inertial>
                    <visual>
                        <origin xyz="0 0 0" rpy="0 0 0" />
                        <geometry>
                            <box size="{lengths[0]} {lengths[1]} {lengths[2]}"/>
                        </geometry>
                        <material name="{material}">
                        <color rgba="{color_rgba}"/>
                    </material>
                    </visual>
                    <collision>
                        <origin xyz="0 0 0" rpy="0 0 0" />
                        <geometry>
                            <box size="{lengths[0]} {lengths[1]} {lengths[2]}"/>
                        </geometry>
                    </collision>
                </link>
        """
        editable = ""
        if coords_disp:
            editable = f"""
                    <link name="X">
                        <visual>
                            <origin xyz="0 0 0" rpy="0 0 0" />
                            <geometry>
                                <box size="{lengths[0] + 0.01} {lengths[1]} {lengths[2]}"/>
                            </geometry>
                            <material name="red">
                            <color rgba="1 0 0 1"/>
                        </material>
                        </visual>
                    </link>
                    <link name="Y">
                        <visual>
                            <origin xyz="0 0 0" rpy="0 0 0" />
                            <geometry>
                                <box size="{lengths[0]} {lengths[1] + 0.01} {lengths[2]}"/>
                            </geometry>
                            <material name="green">
                            <color rgba="0 1 0 1"/>
                        </material>
                        </visual>
                    </link>
                    <link name="Z">
                        <visual>
                            <origin xyz="0 0 0" rpy="0 0 0" />
                            <geometry>
                                <box size="{lengths[0]} {lengths[1]} {lengths[2] + 0.01}"/>
                            </geometry>
                            <material name="blue">
                            <color rgba="0 0 1 1"/>
                        </material>
                        </visual>
                    </link>

                    <joint name="X_joint" type="fixed">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <parent link="base_link"/>
                        <child link="X"/>
                    </joint>
                    <joint name="Y_joint" type="fixed">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <parent link="base_link"/>
                        <child link="Y"/>
                    </joint>
                    <joint name="Z_joint" type="fixed">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <parent link="base_link"/>
                        <child link="Z"/>
                    </joint>

                    <link name="XV">
                        <visual>
                            <origin xyz="0.1 0 0" rpy="0 0 0" />
                            <geometry>
                                <box size="0.2 0.01 0.01"/>
                            </geometry>
                            <material name="red">
                            <color rgba="1 0 0 1"/>
                        </material>
                        </visual>
                    </link>
                    <link name="YV">
                        <visual>
                            <origin xyz="0 0.1 0" rpy="0 0 0" />
                            <geometry>
                                <box size="0.01 0.2 0.01"/>
                            </geometry>
                            <material name="green">
                            <color rgba="0 1 0 1"/>
                        </material>
                        </visual>
                    </link>
                    <link name="ZV">
                        <visual>
                            <origin xyz="0 0 0.1" rpy="0 0 0" />
                            <geometry>
                                <box size="0.01 0.01 0.2"/>
                            </geometry>
                            <material name="blue">
                            <color rgba="0 0 1 1"/>
                        </material>
                        </visual>
                    </link>

                    <joint name="X_jointV" type="fixed">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <parent link="base_link"/>
                        <child link="XV"/>
                    </joint>
                    <joint name="Y_jointV" type="fixed">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <parent link="base_link"/>
                        <child link="YV"/>
                    </joint>
                    <joint name="Z_jointV" type="fixed">
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <parent link="base_link"/>
                        <child link="ZV"/>
                    </joint>
            """

        end = """
            </robot>
            """

        return base + editable + end

    def srdf(self, lengths, side_handles=False) -> str:
        """
        this function generates text for .srdf file with given parameters to create handles and contact surfaces

        :param lengths: [width in x, width in y, width in z]
        :param side_handles: If true, additional handles which are placed at the sides of the cuboid will be added
        :return: text of .srdf file with handles and contact surfaces for the cuboid object
        """

        dist = [0, 0, 0]  # is the distance of the grip from the center of the cuboid
        for i, length in enumerate(lengths):
            if length > 0.07:
                dist[i] = abs(0.07 - length) / 2
        a, b, c = [l / 2 for l in lengths]

        preinfo = f"""
                <?xml version="1.0"?>
                <robot name="box">
                """

        handles1 = ""
        handles2 = ""
        handles3 = ""

        if lengths[0] < 0.075:
            self.h1_active = True
            handles1 = f"""

                 <handle name="handleZpx" clearance="{lengths[0]}">
                    <position> 0 0 {dist[2]}   0.7071068, 0, 0.7071068, 0</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleZmx" clearance="{lengths[0]}">
                    <position> 0 0 {-dist[2]}  0.7071067811865476 0.0 -0.7071067811865475 0.0  </position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleYpx" clearance="{lengths[0]}">
                    <position> 0 {dist[1]} 0.0   0.5 0.5 -0.5  -0.5  </position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleYmx" clearance="{lengths[0]}">
                    <position> 0 {-dist[1]} 0.0   0.5 -0.5 -0.5 0.5  </position>
                    <link name="base_link"/>
                  </handle>

                """
            if side_handles:
                handles1 += f"""
                  <handle name="handleZpxSm" clearance="{lengths[0]}">
                    <position> 0 {-dist[1]} {dist[2]}   0.65328148 0.27059805 0.65328148 0.27059805</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleZpxSp" clearance="{lengths[0]}">
                    <position> 0 {dist[1]} {dist[2]}   0.65328148  -0.27059805  0.65328148 -0.27059805</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleZmxSm" clearance="{lengths[0]}">
                    <position> 0 {-dist[1]} {-dist[2]}    0.65328148  -0.27059805 -0.65328148  0.27059805</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleZmxSp" clearance="{lengths[0]}">
                    <position> 0 {dist[1]} {-dist[2]}  0.65328148  0.27059805 -0.65328148 -0.27059805</position>
                    <link name="base_link"/>
                  </handle>
                            """

        if lengths[1] < 0.075:
            self.h2_active = True
            handles2 = f"""

                <handle name="handleZpy" clearance="{lengths[1]}">
                    <position> 0 0 {dist[2]}   0.5 -0.5  0.5  0.5 </position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleZmy" clearance="{lengths[1]}">
                    <position> 0 0 {-dist[2]}   0.5 0.5 -0.5 0.5  </position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleXmy" clearance="{lengths[1]}">
                    <position> {-dist[0]} 0 0.0   0.7071067811865476 -0.7071067811865475 0.0 0.0  </position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleXpy" clearance="{lengths[1]}">
                    <position> {dist[0]} 0 0.0   0 0 -0.7071067811865475 0.7071067811865476 </position>
                    <link name="base_link"/>
                  </handle>

                """
            if side_handles:
                handles2 += f"""
                <handle name="handleZpySm" clearance="{lengths[1]}">
                    <position> {-dist[0]} 0 {dist[2]}    0.65328148 -0.65328148  0.27059805  0.27059805</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleZpySp" clearance="{lengths[1]}">
                    <position> {dist[0]} 0 {dist[2]}   0.27059805  -0.27059805  0.65328148  0.65328148</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleZmySm" clearance="{lengths[1]}">
                    <position> {-dist[0]} 0 {-dist[2]}   0.65328148  0.65328148 -0.27059805  0.27059805</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleZmySp" clearance="{lengths[1]}">
                    <position> {dist[0]} 0 {-dist[2]}   0.27059805  0.27059805 -0.65328148  0.65328148</position>
                    <link name="base_link"/>
                  </handle>
                  """
        if lengths[2] < 0.075:
            self.h3_active = True
            handles3 = f"""

                <handle name="handleYpz" clearance="{lengths[2]}">
                    <position> 0 {dist[1]} 0.0   0.7071067811865476 0.0 0.0 -0.7071067811865475 </position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleYmz" clearance="{lengths[2]}">
                    <position> 0 {-dist[1]} 0.0   0 0.7071067811865476 0.7071067811865475 0  </position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleXmz" clearance="{lengths[2]}">
                    <position> {-dist[0]} 0 0.0   0, 1, 0, 0 </position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleXpz" clearance="{lengths[2]}">
                    <position> {dist[0]} 0 0.0   0 0 1 0 </position>
                    <link name="base_link"/>
                  </handle>

                """
            if side_handles:
                handles3 += f"""
                <handle name="handleYpzSm" clearance="{lengths[2]}">
                    <position> {-dist[0]} {dist[1]} 0  0  0.9238795 -0.3826834 0</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleYpzSp" clearance="{lengths[2]}">
                    <position> {dist[0]} {dist[1]} 0.0    0 0.3826834 -0.9238795 0</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleYmzSm" clearance="{lengths[2]}">
                    <position> {-dist[0]} {-dist[1]} 0.0   0  0.9238795 0.3826834 0</position>
                    <link name="base_link"/>
                  </handle>

                  <handle name="handleYmzSp" clearance="{lengths[2]}">
                    <position> {dist[0]} {-dist[1]} 0.0    0 0.3826834 0.9238795 0</position>
                    <link name="base_link"/>
                  </handle>
                """

        afterinfo = f"""
                  <contact name="box_surface">
                    <link name="base_link"/>
                     <point>
                         {-a} {-b} {-c}  {-a} {b} {-c}  {-a} {-b} {c}  {-a} {b} {c}
                         {a} {-b} {-c}   {a} {b} {-c}   {a} {-b} {c}   {a} {b} {c}
                     </point>
                     <shape>4 1 5 4 0     4 2 6 7 3     4 2 3 1 0     4 4 5 7 6     4 4 6 2 0     4 1 3 7 5 </shape>
                  </contact>
                </robot>
               """

        return preinfo + handles1 + handles2 + handles3 + afterinfo

    def handles(self, name):
        """
        This function returns list of curently active handles.
        Handle description following:
                    HandleAbc(Sd)
                        A - X/Y/Z from which coordinate will the gripper come from

                        b - m/p if the approach is from minus (m) or plus (p)

                        c - x/y/z is coordinate in which the width of the grip is represented

                        S - optional S means that the handle is a side handle

                        d - m/p argument tells if the side handle is on minus or plus side of cuboid on the third axis

        Warning: if there is unreachable handle, constraint graph validation will be incorrect!

        :param name: argument which is the name of the handled object passed to HPP corbaserver
        :return: list of handles [obj_name/handleAbc, obj_name/handleAbc, ...]
        """

        handle_list = []

        if self.h1_active:
            handle_list = handle_list + [name + "/handleZpx", name + "/handleZmx", name + "/handleYmx",
                                         name + "/handleYpx"]
            if self.side_handles:
                handle_list += [name + "/handleZpxSm", name + "/handleZmxSm", name + "/handleZpxSp",
                                name + "/handleZmxSp"]
        if self.h2_active:
            handle_list = handle_list + [name + "/handleZpy", name + "/handleZmy", name + "/handleXmy",
                                         name + "/handleXpy"]
            if self.side_handles:
                handle_list += [name + "/handleZpySm", name + "/handleZmySm", name + "/handleZpySp",
                                name + "/handleZmySp"]
        if self.h3_active:
            handle_list = handle_list + [name + "/handleYpz", name + "/handleYmz", name + "/handleXmz",
                                         name + "/handleXpz"]
            if self.side_handles:
                handle_list += [name + "/handleYpzSm", name + "/handleYmzSm", name + "/handleYpzSp",
                                name + "/handleYmzSp"]

        return handle_list

    def contact_surfaces(self, name):
        """
        This function returns the name of the contact surface of the object

        :param name: argument which is the name of the handled object passed to HPP corbaserver
        :return: name as string "name/box_surface"
        """
        surface = name + "/box_surface"
        return surface

    def __del__(self):
        os.unlink(self.urdfFilename)
        os.unlink(self.srdfFilename)
