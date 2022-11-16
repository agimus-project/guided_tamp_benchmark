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
        with open("data/" + task_name + "_" + str(demo_id) + ".pkl", 'rb') as f:
            data = pickle.load(f)
        demo = Demonstration()
        demo.object_ids = data["object_ids"]
        demo.objects_poses = data["objects_poses"]
        demo.contacts = data["contacts"]
        demo.furniture_ids = data["furniture_ids"]
        demo.furniture_poses = data["furniture_poses"]
        with open("data/" + task_name + "_" + str(demo_id) + "_" + robot_name + "_poses.pkl", 'rb') as f:
            robot_data = pickle.load(f)
        demo.robot_pose = robot_data[pose_id]
        return demo

    def save(self, demo_file, robot_name, pose_id):
        demo = dict(object_ids=[], object_poses=[], contacts=[], furniture_ids=[], furniture_poses=[])
        demo["object_ids"] = self.object_ids
        demo["objects_poses"] = self.objects_poses
        demo["contacts"] = self.contacts
        demo["furniture_ids"] = self.furniture_ids
        demo["furniture_poses"] = self.furniture_poses
        with open("data/" + demo_file + '.pkl', 'wb') as handle:
            pickle.dump(demo, handle, protocol=pickle.HIGHEST_PROTOCOL)
        robot = dict(pose_id=[])
        robot[pose_id] = self.robot_pose
        with open("data/" + demo_file + "_" + robot_name + '_poses.pkl', 'wb') as handle:
            pickle.dump(robot, handle, protocol=pickle.HIGHEST_PROTOCOL)


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

    dummy = {"object_ids": ["cuboid_0.06_0.06_0.06", "cuboid_0.06_0.06_0.06"], "objects_poses": [a, c],
             "contacts": [b, d], "furniture_ids": ["table"], "furniture_poses": [np.eye(4)]}
    robot_dummy = {"robot_pose": [np.eye(4)]}

    with open('data/dummy_0.pkl', 'wb') as handle:
        pickle.dump(dummy, handle, protocol=pickle.HIGHEST_PROTOCOL)
    with open('data/dummy_0_panda_robot_poses.pkl', 'wb') as handle:
        pickle.dump(robot_dummy, handle, protocol=pickle.HIGHEST_PROTOCOL)

    demo = Demonstration.load('dummy', 0, "panda_robot", "robot_pose")
    demo.save('shelf_1', "panda_robot", "robot_pose")
    demo2 = Demonstration.load("shelf", 1, "panda_robot", "robot_pose")
