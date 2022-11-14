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

    def __init__(self, objects, furniture_objects):
        """

        :param objects: dictionary {'obj_type': ['cuboid', 'ycbv'], 'obj_param': ['id', [width_x, width_y, width_z]], 'obj_pose': [np.array, np.array]}
        :param furniture_objects: dictionary {'obj_type': ['shelf', 'table', 'tunnel'], 'obj_param': [None, [width_x, width_y, width_z]], 'obj_pose': [np.array, np.array, np.array]}
        """
        list_objects = []
        for i, o in enumerate(objects["obj_types"]):
            if o == "cuboid":
                list_objects.append(Cuboid(objects["obj_param"][i]))
            elif o == "ycbv":
                list_objects.append(ObjectYCBV(objects["obj_param"][i]))

        list_furniture = []
        for i, f in enumerate(furniture_objects["obj_types"]):
            r = R.from_matrix(furniture_objects["obj_param"][i][:3, :3])
            if f == "table":
                list_furniture.append(Table(position=furniture_objects["obj_param"][i][:3, 3].tolist(),
                                            rpy=r.as_euler("xyz").tolist(),
                                            desk_size=furniture_objects["obj_param"][i]))
            elif f == "shelf":
                list_furniture.append(Shelf(position=furniture_objects["obj_param"][i][:3, 3].tolist(),
                                            rpy=r.as_euler("xyz").tolist()))
            elif f == "tunnel":
                list_furniture.append(Tunnel(position=furniture_objects["obj_param"][i][:3, 3].tolist(),
                                             rpy=r.as_euler("xyz").tolist()))


    def get_object_poses(self):
        pass

    def get_contacts(self):
        pass

    def get_robot_pose(self):
        pass