#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from typing import List
import pinocchio as pin
from robomeshcat import Robot, Scene, Object

from guided_tamp_benchmark.models.utils import get_models_data_directory
from guided_tamp_benchmark.tasks.base_task import BaseTask

from guided_tamp_benchmark.core import Configuration


class RobotWithTexture(Robot):
    def __init__(self, texture, *args, **kwargs) -> None:
        self.texture = texture
        super().__init__(*args, **kwargs)

    def _init_objects(self, overwrite_color=False):
        pin.forwardKinematics(self._model, self._data, self._q)
        pin.updateGeometryPlacements(
            self._model, self._data, self._geom_model, self._geom_data
        )
        base = pin.SE3(self._pose)
        for g, f in zip(self._geom_model.geometryObjects, self._geom_data.oMg):
            kwargs = dict(
                name=f"{self.name}/{g.name}",
                color=g.meshColor[:3] if not overwrite_color else self._color,
                opacity=g.meshColor[3] if self._opacity is None else self._opacity,
                texture=self.texture,
                pose=(base * f).homogeneous,
            )
            if g.meshPath == "BOX":
                self._objects[kwargs["name"]] = Object.create_cuboid(
                    lengths=2 * g.geometry.halfSide, **kwargs
                )
            elif g.meshPath == "SPHERE":
                self._objects[kwargs["name"]] = Object.create_sphere(
                    radius=g.geometry.radius, **kwargs
                )
            elif g.meshPath == "CYLINDER":
                radius, length = g.geometry.radius, 2 * g.geometry.halfLength
                self._objects[kwargs["name"]] = Object.create_cylinder(
                    radius=radius, length=length, **kwargs
                )
            else:
                self._objects[kwargs["name"]] = Object.create_mesh(
                    path_to_mesh=g.meshPath, scale=g.meshScale, **kwargs
                )


class Renderer:
    def __init__(self, task: BaseTask, pddl=False, scene=None) -> None:
        """Create a renderer for a given task.
        Arguments:
             task: task with the scene elements description
             pddl: boolean indicating if name _pddl.urdf should be used for a robot
             scene: optional robomeshcat scene that avoids creating a new meshcat
                visualizer.
        """
        super().__init__()
        self.task = task
        self.scene = Scene() if scene is None else scene
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
            texture_path = (
                str(get_models_data_directory())
                + "/ycbv/meshes/"
                + o.name
                + "_texture.png"
            )
            vo = RobotWithTexture(
                urdf_path=o.urdfFilename,
                mesh_folder_path=get_models_data_directory(),
                texture=texture_path,
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

    def animate_subgoals(self, fps: int = 1):
        """Create an animation from the demonstration file, that contains the motion of
        object but not the motion of the robot"""
        with self.scene.animation(fps=fps):
            # self.robot.pos = [-5, -5, 0]
            self.robot[:] = self.task.robot.initial_configuration()
            for object_poses in self.task.demo.subgoal_objects_poses.transpose(
                1, 0, 2, 3
            ):
                for obj, pose in zip(self.objects, object_poses):
                    obj.pose = pose
                self.scene.render()

    def animate_path(self, path: List[Configuration], fps: int = 25):
        """Animate the path by setting the robot configuration and object poses."""
        with self.scene.animation(fps=fps):
            for c in path:
                self.render_configuration(c)

    def render_configuration(self, c: Configuration):
        """Update the robot pose and object poses, call render afterward."""
        self.robot[:] = c.q
        for pose, obj in zip(c.poses, self.objects):
            obj.pose = pose
        self.scene.render()
