#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
import pickle
from typing import Optional, List

import numpy as np


class Demonstration:
    def __init__(self):
        self.task_name: Optional[str] = None
        self.demo_id: Optional[int] = None
        self.robot_name: Optional[str] = None
        self.pose_id: Optional[str] = None
        self.object_ids: Optional[List[str]] = None  # can be a list of e.g. YCBV_01 or CUBOID_0.1_0.2_0.8
        self.objects_poses: Optional[np.array] = None  # n*t*4x4 numpy array
        self.contacts: Optional[List[List[int]]] = None
        self.robot_pose: Optional[np.array] = None  # 4x4 numpy array
        self.furniture_ids: Optional[List[str]] = None
        self.furniture_poses: Optional[np.array] = None

    @staticmethod
    def load(task_name, demo_id, robot_name, pose_id):
        data = pickle.load(open("data/" + task_name + "_" + str(demo_id) + ".pkl", 'rb'))
        demo = Demonstration()
        demo.task_name = task_name
        demo.demo_id = demo_id
        demo.robot_name = robot_name
        demo.pose_id = pose_id
        demo.object_ids = data["object_ids"]
        demo.objects_poses = data["objects_poses"]
        demo.contacts = data["contacts"]
        demo.furniture_ids = data["furniture_ids"]
        demo.furniture_poses = data["furniture_poses"]
        robot_data = pickle.load(open("data/" + task_name + "_" + str(demo_id) + "_" + robot_name + "_poses.pkl", 'rb'))
        demo.robot_pose = robot_data[pose_id]
        return demo

    def save(self, overwrite=False):
        demo_file_name = "data/" + self.task_name + "_" + str(self.demo_id) + ".pkl"
        if os.path.exists(demo_file_name) and not overwrite:
            print('The demo file exists, not updating. Please, remove it manually before saving.')
        else:
            demo = dict(object_ids=[], object_poses=[], contacts=[], furniture_ids=[], furniture_poses=[])
            demo["object_ids"] = self.object_ids
            demo["objects_poses"] = self.objects_poses
            demo["contacts"] = self.contacts
            demo["furniture_ids"] = self.furniture_ids
            demo["furniture_poses"] = self.furniture_poses
            pickle.dump(demo, open(demo_file_name, 'wb'), protocol=pickle.HIGHEST_PROTOCOL)

        robot_data_filename = "data/" + self.task_name + "_" + str(self.demo_id) + "_" + self.robot_name + "_poses.pkl"
        robot_poses = {}
        if os.path.exists(robot_data_filename):
            robot_poses = pickle.load(open(robot_data_filename, 'rb'))
        if self.pose_id in robot_poses and not overwrite:
            print('The pose is already in the robot poses, not updating.')
        else:
            robot_poses[self.pose_id] = self.robot_pose
            pickle.dump(robot_poses, open(robot_data_filename, 'wb'))


if __name__ == "__main__":
    a = np.array([[[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.031], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.031], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.041], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.051], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.061], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.071], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.15], [0., 0., 1., 0.081], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.14], [0., 0., 1., 0.081], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.13], [0., 0., 1., 0.081], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.12], [0., 0., 1., 0.081], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.11], [0., 0., 1., 0.081], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.081], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.071], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.061], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.051], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.041], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.031], [0., 0., 0., 1.]],
                  [[1., 0., 0., 0.15], [0., 1., 0., 0.10], [0., 0., 1., 0.031], [0., 0., 0., 1.]]])
    b = [0] + 16 * [1] + [0]

    c = np.array([[[1., 0., 0., 0.15], [0., 1., 0., -0.15], [0., 0., 1., 0.031], [0., 0., 0., 1.]]] * 18)
    d = [0] * 18

    dummy = {"object_ids": ["cuboid_0.06_0.06_0.06", "cuboid_0.06_0.06_0.06"], "objects_poses": np.array([a, c]),
             "contacts": [b, d], "furniture_ids": ["table"], "furniture_poses": np.array([np.eye(4)])}
    robot_dummy = {"robot_pose_0": np.eye(4)}

    with open('data/dummy_0.pkl', 'wb') as handle:
        pickle.dump(dummy, handle, protocol=pickle.HIGHEST_PROTOCOL)
    with open('data/dummy_0_panda_robot_poses.pkl', 'wb') as handle:
        pickle.dump(robot_dummy, handle, protocol=pickle.HIGHEST_PROTOCOL)

    demo = Demonstration.load('dummy', 0, "panda_robot", "robot_pose_0")
    demo.save('shelf_0', "panda_robot", "robot_pose_0")
    demo2 = Demonstration.load("shelf", 0, "panda_robot", "robot_pose_0")
