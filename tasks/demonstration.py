#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>


import pickle
from typing import Optional, List

import numpy as np


class Demonstration:
    def __init__(self):
        self.task_name = None
        self.demo_id = None
        self.robot_name = None
        self.pose_id = None

        self.object_ids: Optional[List[str]] = None  # can be a list of e.g. YCBV_01 or CUBOID_0.1_0.2_0.8
        # todo: add other types as well
        self.objects_poses = None
        self.contacts: Optional[List[List[int]]] = None
        self.robot_pose = None
        self.furniture_ids = None
        self.furniture_poses = None

    def get_object_poses(self):
        return self.objects_poses

    def get_contacts(self):
        return self.contacts

    def get_robot_pose(self):
        return self.robot_pose

    @staticmethod
    def load(task_name, demo_id, robot_name, pose_id):
        with open("data/" + task_name + "_" + demo_id + ".pkl", 'rb') as f:
            data = pickle.load(f)
        Demonstration.object_ids = data["object_ids"]
        Demonstration.objects_poses = data["objects_poses"]
        Demonstration.contacts = data["contacts"]
        Demonstration.furniture_ids = data["furniture_ids"]
        Demonstration.furniture_poses = data["furniture_poses"]
        with open("data/" + task_name + "_" + demo_id + "_" + robot_name + "_poses.pkl", 'rb') as f:
            robot_data = pickle.load(f)
        Demonstration.robot_pose = robot_data[pose_id]
        return Demonstration

    def save(self, demo_file, robot_name, pose_id):
        with open(demo_file, 'rb') as f:
            data = pickle.load(f)
        self.object_ids = data["object_ids"]
        self.objects_poses = data["objects_poses"]
        self.contacts = data["contacts"]
        self.robot_pose = data["robot_pose"][robot_name][pose_id]
        self.furniture_ids = data["furniture_ids"]
        self.furniture_poses = data["furniture_poses"]


if __name__ == "__main__":
    a = [np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.031], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.031], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.041], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.051], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.061], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.071], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.081], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.14], [0., 0., 1., 0.081], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.13], [0., 0., 1., 0.081], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.12], [0., 0., 1., 0.081], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.11], [0., 0., 1., 0.081], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.081], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.071], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.061], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.051], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.041], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.031], [0., 0., 0., 1.]]),
         np.array([[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.031], [0., 0., 0., 1.]])]
    b = [0] + 16 * [1] + [0]

    c = [np.array([[1., 0., 0., 0.15], [0., 1., 0., -0.15], [0., 0., 1., 0.031], [0., 0., 0., 1.]])] * 18
    d = [0] * 18

    dummy = {"object_ids": [[0.06, 0.06, 0.06], [0.06, 0.06, 0.06]], "objects_poses": [a, c], "contacts": [b, d],
             "robot_pose": {"panda_robot": [np.eye(4)]}, "furniture_ids": ["table"], "furniture_poses": [np.eye(4)]}

    with open('dummy_demo.pkl', 'wb') as handle:
        pickle.dump(dummy, handle, protocol=pickle.HIGHEST_PROTOCOL)

    demo = Demonstration()
    demo.save('dummy_demo.pkl', "panda_robot", 0)
