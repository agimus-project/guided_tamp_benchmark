#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>


import numpy as np
from scipy.spatial.transform import Rotation as R

from models.objects.cuboid import Cuboid
from models.objects.object_ycbv import ObjectYCBV

from models.furniture.table import Table
from models.furniture.shelf import Shelf
from models.furniture.tunnel import Tunnel

class Demo():

    def __init__(self, objects, furniture_objects, robot_pose):
        """

        :param objects: dictionary {'obj_type': ['cuboid', 'ycbv'], 'obj_param': ['id', [width_x, width_y, width_z]], 'obj_poses': [[np.array,...], [np.array,...],...]}
        :param furniture_objects: dictionary {'obj_type': ['shelf', 'table', 'tunnel'], 'obj_param': [None, [width_x, width_y, width_z], [[width in x, width in y, width in z], thickness]], 'obj_pose': [np.array, np.array, np.array]}
        :param robot_pose: np.array(4x4)
        """

        assert all([len(furniture_objects["obj_pose"][0]) == len(T) for T in furniture_objects["obj_poses"]])

        self.list_objects = []
        for i, o in enumerate(objects["obj_types"]):
            if o == "cuboid":
                self.list_objects.append(Cuboid(objects["obj_param"][i]))
            elif o == "ycbv":
                self.list_objects.append(ObjectYCBV(objects["obj_param"][i]))

        self.list_furniture = []
        for i, f in enumerate(furniture_objects["obj_types"]):
            r = R.from_matrix(furniture_objects["obj_pose"][i][:3, :3])
            if f == "table":
                self.list_furniture.append(Table(position=furniture_objects["obj_pose"][i][:3, 3].tolist(),
                                                 rpy=r.as_euler("xyz").tolist(),
                                                 desk_size=furniture_objects["obj_param"][i]))
            elif f == "shelf":
                self.list_furniture.append(Shelf(position=furniture_objects["obj_pose"][i][:3, 3].tolist(),
                                                 rpy=r.as_euler("xyz").tolist()))
            elif f == "tunnel":
                self.list_furniture.append(Tunnel(position=furniture_objects["obj_pose"][i][:3, 3].tolist(),
                                                  rpy=r.as_euler("xyz").tolist(),
                                                  lengths=furniture_objects["obj_param"][i][0],
                                                  tunnel_walls_thickness=furniture_objects["obj_param"][i][1]))

        self.obj_poses = objects["obj_poses"]
        self.robot_pose = robot_pose


    def get_object_poses(self, obj_id):
        return self.obj_poses[obj_id]

    def get_contacts(self, obj_id):
        return self.list_objects[obj_id].handles()

    def get_robot_pose(self):
        return self.robot_pose