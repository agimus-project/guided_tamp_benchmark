#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>


import pickle
import numpy as np


class Demo():
    def __init__(self):
        self.object_ids = None
        self.objects_poses = None
        self.contacts = None
        self.robot_pose = None

    def get_object_poses(self):
        return self.objects_poses

    def get_contacts(self):
        return self.contacts

    def get_robot_pose(self):
        return self.robot_pose

    def load(self, demo_file):
        with open(demo_file, 'rb') as f:
            data = pickle.load(f)
        self.object_ids = data["object_ids"]
        self.objects_poses = data["objects_poses"]
        self.contacts = data["contacts"]
        self.robot_pose = data["robot_pose"]
        return self

    def save(self, demo_file):
        with open(demo_file, 'rb') as f:
            data = pickle.load(f)
        self.object_ids = data["object_ids"]
        self.objects_poses = data["objects_poses"]
        self.contacts = data["contacts"]
        self.robot_pose = data["robot_pose"]


a = [np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.15],[0., 0., 1., 0.031],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.15],[0., 0., 1., 0.031],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.15],[0., 0., 1., 0.041],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.15],[0., 0., 1., 0.051],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.15],[0., 0., 1., 0.061],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.15],[0., 0., 1., 0.071],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.15],[0., 0., 1., 0.081],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.14],[0., 0., 1., 0.081],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.13],[0., 0., 1., 0.081],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.12],[0., 0., 1., 0.081],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.11],[0., 0., 1., 0.081],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.10],[0., 0., 1., 0.081],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.10],[0., 0., 1., 0.071],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.10],[0., 0., 1., 0.061],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.10],[0., 0., 1., 0.051],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.10],[0., 0., 1., 0.041],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.10],[0., 0., 1., 0.031],[0., 0., 0., 1.]]),
     np.array([[1., 0., 0., 0.15],[0., 1., 0., 0.10],[0., 0., 1., 0.031],[0., 0., 0., 1.]])]
b = [0] + 16 * [1] + [0]

c = [np.array([[1., 0., 0., 0.15],[0., 1., 0., -0.15],[0., 0., 1., 0.031],[0., 0., 0., 1.]])] * 18
d = [0] * 18

dummy = {"object_ids": [[0.06, 0.06, 0.06], [0.06, 0.06, 0.06]], "objects_poses": [a, c], "contacts": [b,d], "robot_pose": [np.eye(4)]}

with open('dummy_demo.pkl', 'wb') as handle:
    pickle.dump(dummy, handle, protocol=pickle.HIGHEST_PROTOCOL)

demo = Demo()
demo.save('dummy_demo.pkl')

