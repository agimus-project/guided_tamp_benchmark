#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 20.02.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from __future__ import print_function
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import sys
import numpy as np

from typing import List

from guided_tamp_benchmark.tasks.base_task import BaseTask
from guided_tamp_benchmark.models.robots.base import BaseRobot
from guided_tamp_benchmark.models.furniture.base import FurnitureObject
from guided_tamp_benchmark.models.objects.base import BaseObject
from guided_tamp_benchmark.tasks.configuration import Configuration


from guided_tamp_benchmark.models.utils import get_models_data_directory
from guided_tamp_benchmark.models.robots.panda_robot import PandaRobot
from guided_tamp_benchmark.models.furniture.table import Table
from guided_tamp_benchmark.models.furniture.shelf import Shelf
from guided_tamp_benchmark.models.objects.object_ycbv import ObjectYCBV

from guided_tamp_benchmark.tasks.shelf_task import ShelfTask

def rename_frames(model: pin.Model, model_name: str):
    """Renames the frame names in given model to {previous name}_{model_name}_{i}."""
    for i, frame in enumerate(model.frames):
        if i == 0:
            continue
        frame.name = f"{frame.name}_{model_name}_{i}"


def rename_joints(model: pin.Model, model_name: str):
    """Renames the joint names in given model to {previous name}_{model_name}_{i}."""
    for i in range(0, len(model.names)):
        if i == 0:
            continue
        model.names[i] = f"{model.names[i]}_{model_name}_{i}"


def create_model(robots: List[BaseRobot], objects: List[BaseObject], furniture: list[FurnitureObject],
                 robot_poses: List[pin.SE3]) -> (pin.Model, pin.GeometryModel):
    """Creates pinocchio urdf model and pinocchio collision model from given robots, furniture and objects."""
    p_r = None
    c_r = None

    for i, model in enumerate(reversed(furniture + objects)):
        if i < len(objects):
            p, c, _ = pin.buildModelsFromUrdf(model.urdfFilename, package_dirs=str(get_models_data_directory()),
                                              root_joint=pin.JointModelFreeFlyer())
        else:
            p, c, _ = pin.buildModelsFromUrdf(model.urdfFilename, package_dirs=str(get_models_data_directory()))
        rename_joints(p, model.name)
        rename_frames(p, model.name)
        c.addAllCollisionPairs()
        if i == 0:
            p_r = p
            c_r = c
            continue
        p_r, c_r = pin.appendModel(p_r, p, c_r, c, 0, pin.SE3(np.eye(3), np.array([0, 0, 0])))

    for i, robot in enumerate(robots):
        p, c, _ = pin.buildModelsFromUrdf(robot.urdfFilename, package_dirs=str(get_models_data_directory()))
        c.addAllCollisionPairs()
        pin.removeCollisionPairs(p, c, robot.srdfFilename)
        rename_joints(p, robot.name)
        rename_frames(p, robot.name)
        p_r, c_r = pin.appendModel(p_r, p, c_r, c, 0, robot_poses[i])

    # c_r.addAllCollisionPairs()
    print("num collision pairs - initial:", len(c_r.collisionPairs))
    # pin.removeCollisionPairs(p_r, c_r, robot.srdfFilename)
    return p_r, c_r


def extract_from_task(task: BaseTask) -> dict:
    """Extracts robots, objects, furniture and robot poses from task and returns it in dictionary"""
    furniture = []
    for i, f in enumerate(task.demo.furniture_ids):
        if f == "table":
            furniture.append(Table(task.demo.furniture_poses[i], **task.demo.furniture_params[i]))
        elif f == "shelf":
            furniture.append(Shelf(task.demo.furniture_poses[i], **task.demo.furniture_params[i]))
    objects = []
    for i, o in enumerate(task.demo.object_ids):
        objects.append(ObjectYCBV(f"obj_0000{o[-2:]}"))

    # TODO: change for multi robot
    if isinstance(task.robot, list):
        robots = task.robot
    else:
        robots = [task.robot]
    robot_poses = [pin.SE3(task.get_robot_pose()[:3, :3], np.squeeze(task.get_robot_pose()[:3, 3:]))]

    return {"robots": robots, "objects": objects, "furniture": furniture, "robot_poses": robot_poses}


def pose_as_matrix_to_pose_as_quat(pose: np.array) -> np.array:
    T = pin.SE3(pose[:3, :3], np.squeeze(pose[:3, 3:]))
    xyz_quat = pin.se3ToXYZQUAT(T)
    return xyz_quat


class Collision:
    """The collision class consists of Pinocchio urdf and collision models and functions for collision checking and
    Pinocchio model rendering."""

    def __init__(self, task: BaseTask):
        """Initilizie with task eg. ShelfTask..."""
        self.pin_mod, self.col_mod = create_model(**extract_from_task(task))
        self.task = task
        # self.robot_pose = pose_as_matrix_to_pose_as_quat(task.get_robot_pose())

    def is_config_valid(self, configuration: Configuration) -> bool:
        """Returns true if given configuration is collision free"""
        config = configuration.to_numpy()

        # Create data structures
        data = self.pin_mod.createData()
        geom_data = pin.GeometryData(self.col_mod)

        # Compute all the collisions
        if pin.computeCollisions(self.pin_mod, data, self.col_mod, geom_data, config, False):
            for k in range(len(self.col_mod.collisionPairs)):
                cr = geom_data.collisionResults[k]
                cp = self.col_mod.collisionPairs[k]
                print("collision pair:", cp.first, ",", cp.second, "- collision:", "Yes" if cr.isCollision() else "No")
            return False
        else:
            return True

    def visualize_through_pinocchio(self, configuration: Configuration):
        """will visualize the given configuration on Pinocchio collision model"""
        config = configuration.to_numpy()
        viz = MeshcatVisualizer(self.pin_mod, self.col_mod, self.col_mod)

        # Initialize the viewer.
        try:
            viz.initViewer(open=True)
        except ImportError as error:
            print(error)
            sys.exit(0)

        try:
            viz.loadViewerModel("shapes")
        except AttributeError as error:
            print(error)
            sys.exit(0)

        viz.display(config)
        input("press enter to continue")


task = ShelfTask(demo_id=2, robot=PandaRobot(), robot_pose_id=3)

collision = Collision(task)

config = Configuration([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4, 0., 0.], task.demo.objects_poses[:,0])

collision.is_config_valid(config)
collision.visualize_through_pinocchio(config)
pass
