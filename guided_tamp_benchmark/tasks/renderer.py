#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from typing import List

from robomeshcat import Robot, Scene

from guided_tamp_benchmark.models.utils import get_models_data_directory
from guided_tamp_benchmark.tasks.base_task import BaseTask

from guided_tamp_benchmark.core import Configuration


class Renderer:
    def __init__(self, task: BaseTask, pddl=False) -> None:
        super().__init__()
        self.task = task
        self.scene = Scene()
        "Add robot into the scene"
        self.robot = Robot(
            urdf_path=task.robot.urdfFilename.replace(".urdf", "_pddl.urdf")
            if pddl
            else task.robot.urdfFilename,
            mesh_folder_path=get_models_data_directory(),
            pose=task.get_robot_pose(),
        )
        self.scene.add_robot(self.robot)

        "Add static furniture into the scene"
        for f in task.furniture:
            self.scene.add_robot(
                Robot(
                    urdf_path=f.urdfFilename,
                    mesh_folder_path=get_models_data_directory(),
                )
            )

        "Add movable objects into the scene"
        self.objects = []
        for o in task.objects:
            vo = Robot(
                urdf_path=o.urdfFilename, mesh_folder_path=get_models_data_directory()
            )
            self.objects.append(vo)
            self.scene.add_robot(vo)

    def animate_demonstration(self, fps: int = 25):
        """Create an animation from the demonstration file, that contains the motion of
        object but not the motion of the robot"""
        with self.scene.animation(fps=fps):
            self.robot[:] = self.task.robot.initial_configuration()
            for object_poses in self.task.demo.objects_poses.transpose(1, 0, 2, 3):
                for obj, pose in zip(self.objects, object_poses):
                    obj.pose = pose
                self.scene.render()

    def animate_path(self, path: List[Configuration], fps: int = 25):
        """Animate the path by setting the robot configuration and object poses."""
        with self.scene.animation(fps=fps):
            for c in path:
                self.robot[:] = c.q
                for pose, obj in zip(c.poses, self.objects):
                    obj.pose = pose
                self.scene.render()
