#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 14.11.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import os
import pickle
from pathlib import Path
from typing import Optional, List

import numpy as np


class Demonstration:
    def __init__(self):
        self.task_name: Optional[str] = None
        self.demo_id: Optional[int] = None
        self.robot_name: Optional[str] = None
        self.pose_id: Optional[str] = None
        self.object_ids: Optional[
            List[str]
        ] = None  # can be a list of e.g. YCBV_01 or CUBOID_0.1_0.2_0.8
        self.objects_poses: Optional[np.array] = None  # n*t*4x4 numpy array
        self.subgoal_objects_poses: Optional[np.array] = None
        self.contacts: Optional[
            np.array
        ] = None  # n * t np array of boolean grasped/not grasped
        self.robot_pose: Optional[np.array] = None  # 4x4 numpy array
        self.furniture_ids: Optional[List[str]] = None
        self.furniture_poses: Optional[np.array] = None
        self.furniture_params = None

    @staticmethod
    def _get_data_directory() -> Path:
        """Get path to the data of the demonstrations/tasks."""
        return Path(__file__).resolve().parent.joinpath("data")

    @staticmethod
    def _get_demonstration_filepath(task_name: str, demo_id: int) -> Path:
        """Get path to the demonstration file."""
        return Demonstration._get_data_directory().joinpath(
            f"{task_name}_{demo_id}.pkl"
        )

    @staticmethod
    def get_demonstration_ids(task_name: str) -> list[int]:
        """Get valid demonstration IDs for a given task name."""
        datadir = Demonstration._get_data_directory()
        return [
            int(d.name[len(task_name) + 1 : -4])
            for d in datadir.iterdir()
            if d.name.startswith(task_name) and not d.name.endswith("poses.pkl")
        ]

    @staticmethod
    def _get_robot_poses_filepath(
        task_name: str, demo_id: int, robot_name: str
    ) -> Path:
        """Get path to the robot_poses file."""
        return Demonstration._get_data_directory().joinpath(
            f"{task_name}_{demo_id}_{robot_name}_poses.pkl"
        )

    @staticmethod
    def get_number_of_robot_poses(task_name: str, demo_id: int, robot_name: str) -> int:
        """Get number of robot poses for a given task_name demonstration id and robot
        name."""
        fn = Demonstration._get_robot_poses_filepath(task_name, demo_id, robot_name)
        robot_data = pickle.load(open(fn, "rb"))
        return len(robot_data)

    @staticmethod
    def load(task_name, demo_id, robot_name, pose_id):
        """Load a demonstration from file and return the instance of the class
        'Demonstration'"""
        data = pickle.load(
            open(Demonstration._get_demonstration_filepath(task_name, demo_id), "rb")
        )
        demo = Demonstration()
        demo.task_name = task_name
        demo.demo_id = demo_id
        demo.robot_name = robot_name
        demo.pose_id = pose_id
        demo.object_ids = data["object_ids"]
        demo.objects_poses = data["objects_poses"]
        if "subgoal_objects_poses" in data.keys():
            demo.subgoal_objects_poses = data["subgoal_objects_poses"]
        demo.contacts = data["contacts"]
        demo.furniture_ids = data["furniture_ids"]
        demo.furniture_poses = data["furniture_poses"]
        demo.furniture_params = data["furniture_params"]
        robot_data = pickle.load(
            open(
                Demonstration._get_robot_poses_filepath(task_name, demo_id, robot_name),
                "rb",
            )
        )
        valid_pose_id = max(0, min(pose_id, len(robot_data) - 1))
        if valid_pose_id != pose_id:
            print(
                f"WARNING: using {valid_pose_id} pose_id instead of requested {pose_id}"
            )
        demo.robot_pose = robot_data[valid_pose_id]
        return demo

    def save(self, overwrite=False):
        """Save the demonstration into the files. Overwrite existing data only if
        @param overwrite is True"""
        demo_file_path = Demonstration._get_demonstration_filepath(
            self.task_name, self.demo_id
        )
        if os.path.exists(demo_file_path) and not overwrite:
            print(
                "The demo file exists, not updating. "
                "Please, remove it manually before saving."
            )
        else:
            demo = dict(
                object_ids=[],
                object_poses=[],
                contacts=[],
                furniture_ids=[],
                furniture_poses=[],
            )
            demo["object_ids"] = self.object_ids
            demo["objects_poses"] = self.objects_poses
            demo["subgoal_objects_poses"] = self.subgoal_objects_poses
            demo["contacts"] = self.contacts
            demo["furniture_ids"] = self.furniture_ids
            demo["furniture_poses"] = self.furniture_poses
            demo["furniture_params"] = self.furniture_params
            pickle.dump(
                demo, open(demo_file_path, "wb"), protocol=pickle.HIGHEST_PROTOCOL
            )

        robot_data_path = Demonstration._get_robot_poses_filepath(
            self.task_name, self.demo_id, self.robot_name
        )
        robot_poses = (
            {}
            if not os.path.exists(robot_data_path)
            else pickle.load(open(robot_data_path, "rb"))
        )
        if self.pose_id in robot_poses and not overwrite:
            print("The pose is already in the robot poses, not updating.")
        else:
            robot_poses[self.pose_id] = self.robot_pose
            pickle.dump(robot_poses, open(robot_data_path, "wb"))


if __name__ == "__main__":
    """Example of creating a demonstration."""
    T = 5
    demo = Demonstration()
    demo.task_name = "test"
    demo.demo_id = 0
    demo.robot_name = "panda"
    demo.pose_id = 0
    demo.object_ids = ["CUBOID_0.1_0.2_0.8", "CUBOID_0.1_0.2_0.8"]
    demo.objects_poses = (
        np.eye(4).reshape(1, 1, 4, 4).repeat(T, axis=1).repeat(2, axis=0)
    )
    demo.subgoal_objects_poses = (
        np.eye(4).reshape(1, 1, 4, 4).repeat(T, axis=1).repeat(2, axis=0)
    )
    demo.contacts = np.zeros((2, T), dtype=bool)
    demo.robot_pose = np.eye(4)
    demo.furniture_ids = ["tunnel"]
    demo.furniture_poses = np.eye(4).reshape(1, 4, 4)
    demo.furniture_params = [
        {
            "lengths": [0.1, 0.2, 0.3],
            "tunnel_walls_thickness": 0.16,
            "collision_walls_thickness": 0.1,
        }
    ]
    demo.save(overwrite=True)

    """Example of loading the demonstration """
    d = Demonstration.load("test", 0, "panda", 0)
