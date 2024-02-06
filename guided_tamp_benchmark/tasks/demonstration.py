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
            # TODO: any reason we init with empty lists?
            demo = dict(
                object_ids=[],
                # object_poses=[],
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
    frame_n = 20
    subgoal_n = 2
    # # 1. Create fake demonstration with 02 object mowing to the left a bit
    # demo = Demonstration()
    # demo.task_name = "shelf"
    # demo.demo_id = 9
    # demo.robot_name = "panda"
    # demo.pose_id = 0
    # demo.object_ids = ["YCBV_02"]
    # start_pose = np.eye(4)
    # start_pose[0, 3] = 0.
    # start_pose[2, 3] = 0.11
    # step = np.zeros((4, 4))
    # step[1, 3] = 0.01
    # end_pose = start_pose - (frame_n - 3) * step
    # object_poses = [start_pose] * 3 + [start_pose - i * step for i in range(frame_n -
    #                                                                        6)]\
    #                + [end_pose] * 3
    # demo.objects_poses = np.array([object_poses])
    # demo.subgoal_objects_poses = np.array([[start_pose, end_pose]])
    # demo.contacts = np.array([[0] * 3 + [1] * (frame_n - 6) + [0] * 3])
    # demo.robot_pose = np.eye(4)
    # demo.robot_pose[0, 3] = -0.38
    # demo.furniture_ids = ["table"]
    # demo.furniture_poses = np.eye(4).reshape(1, 4, 4)
    # demo.furniture_params = [{'desk_size': [1.5, 1.0, 0.75], 'leg_display': True}]
    # demo.save(overwrite=True)

    # from renderer import Renderer
    # from shelf_task import ShelfTask
    # from guided_tamp_benchmark.models.robots.panda_robot import PandaRobot
    # import time
    # task = ShelfTask(9, PandaRobot(), 0)
    # r = Renderer(task)
    # r.animate_demonstration()
    # time.sleep(5)
    # # """Example of loading the demonstration """
    # # d = Demonstration.load("test", 0, "panda", 0)
    #
    # Convert old demo to new demo
    demo = Demonstration()
    old_demo_path = demo._get_data_directory().joinpath('demo_08.pkl')
    old_demo = pickle.load(open(old_demo_path, 'rb'))

    ##################################################
    # To fill manually

    demo.task_name = "shelf"
    demo.demo_id = 8
    demo.robot_name = "panda"
    demo.pose_id = 0
    obj_grasp_release = [[(83, 123)], [(18, 66)]]
    demo.robot_pose = np.eye(4)
    demo.robot_pose[0, 3] = -0.38
    demo.furniture_ids = ["table", "shelf"]
    demo.furniture_params = [
        {'desk_size': [1.5, 0.7, 0.78], 'leg_display': True},
        {'display_inside_shelf': True}
    ]
    table_pose = np.eye(4)
    shelf_pose = np.eye(4)
    shelf_pose[:3, 3] = [0., 0.6, 0.415]
    shelf_pose[:3, :3] = np.array([[0., -1., 0.],
                                   [1., 0., 0.],
                                   [0., 0., 1.]])
    demo.furniture_poses = np.array([table_pose, shelf_pose])
    ##################################################

    demo.object_ids = [obj_id.upper() for obj_id in old_demo['object_ids']]
    demo.objects_poses = old_demo['full_obj_poses']
    demo.objects_poses[:, :, 2, 3] -= 0.03
    demo.contacts = np.zeros(old_demo['full_obj_poses'].shape[:2])
    for obj_id, grasp_rel_seq in enumerate(obj_grasp_release):
        for grasp, release in grasp_rel_seq:
            demo.contacts[obj_id, grasp:release] = 1
    demo.save(overwrite=True)


    # task = ShelfTask(8, PandaRobot(), 0)
    # r = Renderer(task)
    # r.animate_demonstration()
    # time.sleep(5)
