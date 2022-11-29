#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from robomeshcat import Robot, Scene

from guided_tamp_benchmark.models.utils import get_models_data_directory
from guided_tamp_benchmark.tasks.base_task import BaseTask
import pinocchio as pin

class Renderer:
    def __init__(self, task: BaseTask) -> None:
        super().__init__()
        self.task = task
        self.scene = Scene()
        'Add robot into the scene'
        self.robot = Robot(urdf_path=task.robot.urdfFilename, mesh_folder_path=get_models_data_directory(),
                           pose=task.get_robot_pose())
        self.scene.add_robot(self.robot)

        'Add static furniture into the scene'
        for f in task.furniture:
            self.scene.add_robot(Robot(urdf_path=f.urdfFilename, mesh_folder_path=get_models_data_directory()))

        'Add movable objects into the scene'
        self.objects = []
        for o in task.objects:
            vo = Robot(urdf_path=o.urdfFilename, mesh_folder_path=get_models_data_directory())
            self.objects.append(vo)
            self.scene.add_robot(vo)

    def animate_demonstration(self, fps: int = 25):
        """Create an animation from the demonstration file, that contains the motion of object but not the motion of
         the robot"""
        with self.scene.animation(fps=fps):
            for object_poses in self.task.demo.objects_poses.transpose(1, 0, 2, 3):
                for obj, pose in zip(self.objects, object_poses):
                    obj.pose = pose
                self.scene.render()

    def animate_path(self, path, robot_ndofs, fps: int = 25):
        with self.scene.animation(fps=fps):
            for i in range(len(path)):
                for j in range(robot_ndofs):
                    self.robot.__setitem__(j, path[i][j])
                for k, obj in enumerate(self.objects):
                    obj.pose = pin.XYZQUATToSE3(path[i][robot_ndofs + k * 7: robot_ndofs + (k + 1) * 7]).homogeneous
                self.scene.render()