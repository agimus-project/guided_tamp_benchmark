#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 20.02.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import sys
import numpy as np

from typing import List

from guided_tamp_benchmark.tasks.configuration import Configuration

from guided_tamp_benchmark.models.robots.base import BaseRobot
from guided_tamp_benchmark.models.objects.base import BaseObject
from guided_tamp_benchmark.models.furniture.tunnel import Tunnel
from guided_tamp_benchmark.models.furniture.base import FurnitureObject

from guided_tamp_benchmark.models.utils import get_models_data_directory


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


def rename_geometry(collision_model: pin.GeometryModel, model_name: str):
    """Renames the joint names in given model to {previous name}_{model_name}_{i}."""
    for i, geom in enumerate(collision_model.geometryObjects):
        geom.name = f"{geom.name[0:-2]}_{model_name}_{i}"


def remove_collisions_for_tunnel(full_coll_mod: pin.GeometryModel, rob_coll_mod: pin.GeometryModel,
                                 disabled_tunnel_links: List[str]):
    """removes collision pairs between all robot links and Tunnel.disabled_robot_collision_for_links"""
    disabled_id = []
    for i, obj in enumerate(full_coll_mod.geometryObjects):
        for disabled in disabled_tunnel_links:
            if obj.name.find(disabled) != -1:
                disabled_id.append(i)

    for obj in rob_coll_mod.geometryObjects:
        for id in disabled_id:
            full_coll_mod.removeCollisionPair(pin.CollisionPair(
                full_coll_mod.getGeometryId(obj.name),
                id
            ))


def create_model(robots: List[BaseRobot], objects: List[BaseObject], furniture: List[FurnitureObject],
                 robot_poses: List[pin.SE3], remove_tunnel_collisions: bool) -> (pin.Model, pin.GeometryModel):
    """Creates pinocchio urdf model and pinocchio collision model from given robots, furniture and objects."""
    p_r = None
    c_r = None

    for i, model in enumerate(reversed(furniture + objects)):
        if i < len(objects):
            p, c, _ = pin.buildModelsFromUrdf(model.urdfFilename, package_dirs=str(get_models_data_directory()),
                                              root_joint=pin.JointModelFreeFlyer())
        else:
            p, c, _ = pin.buildModelsFromUrdf(model.urdfFilename, package_dirs=str(get_models_data_directory()))

        rename_joints(p, model.name + f"_{i}")
        rename_frames(p, model.name + f"_{i}")
        rename_geometry(c, model.name + f"_{i}")
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
        rename_joints(p, robot.name + f"_{i}")
        rename_frames(p, robot.name + f"_{i}")
        rename_geometry(c, robot.name + f"_{i}")
        p_r, c_r = pin.appendModel(p_r, p, c_r, c, 0, robot_poses[i])
        if remove_tunnel_collisions:
            for i, model in enumerate(reversed(furniture)):
                if model.name == "tunnel":
                    suffix = "_" + model.name + f"_{i + len(objects)}"
                    remove_collisions_for_tunnel(c_r, c, [s + suffix for s in model.disabled_collision_links_for_robot])

    print("num collision pairs - initial:", len(c_r.collisionPairs))
    return p_r, c_r


def extract_from_task(task) -> dict:
    """Extracts robots, objects, furniture and robot poses from task and returns it in dictionary"""
    furniture = task.furniture
    objects = task.objects
    for i, obj in enumerate(objects):
        if obj.name == "tray":
            objects.pop(i)

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

    def __init__(self, task):
        """Initilizie with task eg. ShelfTask..."""
        self.task = task
        self.tunnel = True if task.task_name == "tunnel" else False
        self.pin_mod, self.col_mod = create_model(remove_tunnel_collisions=self.tunnel, **extract_from_task(task))

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


if __name__ == '__main__':
    from guided_tamp_benchmark.models.robots import *
    from guided_tamp_benchmark.tasks import *

    task = WaiterTask(demo_id=0, robot=KukaMobileIIWARobot(), robot_pose_id=0)

    collision = Collision(task)

    config = Configuration(task.robot.initial_configuration(),
                           task.demo.objects_poses[:3, 0])

    collision.is_config_valid(config)
    collision.visualize_through_pinocchio(config)
    pass
