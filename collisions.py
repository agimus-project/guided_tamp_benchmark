#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 20.02.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from __future__ import print_function
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import sys
import numpy as np

from guided_tamp_benchmark.tasks.base_task import BaseTask
from guided_tamp_benchmark.tasks.configuration import Configuration


from guided_tamp_benchmark.models.utils import get_models_data_directory
from guided_tamp_benchmark.models.robots.panda_robot import PandaRobot
from guided_tamp_benchmark.models.furniture.table import Table
from guided_tamp_benchmark.models.furniture.shelf import Shelf
from guided_tamp_benchmark.models.objects.object_ycbv import ObjectYCBV

from guided_tamp_benchmark.tasks.shelf_task import ShelfTask


def rename_frames(model, model_name):
    for i, frame in enumerate(model.frames):
        if i == 0:
            continue
        frame.name = f"{model_name}{i}"


def rename_joints(model, model_name):
    for i in range(0, len(model.names)):
        if i == 0:
            continue
        model.names[i] = f"{model_name}{i}"


def create_model(robot, objects, furniture):
    p_r, c_r, _ = pin.buildModelsFromUrdf(robot.urdfFilename, package_dirs=str(get_models_data_directory()),
                                          root_joint=pin.JointModelFreeFlyer())

    for i, model in enumerate(objects + furniture):
        if i < len(objects):
            p, c, _ = pin.buildModelsFromUrdf(model.urdfFilename, package_dirs=str(get_models_data_directory()),
                                              root_joint=pin.JointModelFreeFlyer())
            # rename_joints(p, model.name)
            rename_frames(p, model.name)
            p.names[1] = f"{model.name}{i}_root_joint"
            p_r, c_r = pin.appendModel(p_r, p, c_r, c, 0, pin.SE3(np.eye(3), np.array([0, 0, 0])))
        else:
            p, c, _ = pin.buildModelsFromUrdf(model.urdfFilename, package_dirs=str(get_models_data_directory()))
            rename_frames(p, model.name)
            p_r, c_r = pin.appendModel(p_r, p, c_r, c, 0, pin.SE3(np.eye(3), np.array([0, 0, 0])))

    c_r.addAllCollisionPairs()
    print("num collision pairs - initial:", len(c_r.collisionPairs))
    pin.removeCollisionPairs(p_r, c_r, robot.srdfFilename)
    return p_r, c_r


def extract_from_task(task):
    furniture = []
    for i, f in enumerate(task.demo.furniture_ids):
        if f == "table":
            furniture.append(Table(task.demo.furniture_poses[i], **task.demo.furniture_params[i]))
        elif f == "shelf":
            furniture.append(Shelf(task.demo.furniture_poses[i], **task.demo.furniture_params[i]))
    objects = []
    for i, o in enumerate(task.demo.object_ids):
        objects.append(ObjectYCBV(f"obj_0000{o[-2:]}"))

    robot = task.robot

    return {"robot": robot, "objects": objects, "furniture": furniture}


def pose_as_matrix_to_pose_as_quat(pose):
    T = pin.SE3(pose[:3, :3], np.squeeze(pose[:3, 3:]))
    xyz_quat = pin.se3ToXYZQUAT(T)
    # quat = list(R.from_matrix(task.demo.objects_poses[:3, :3]).as_quat())
    # t = list(np.squeeze(task.demo.objects_poses[:3, 3:]))
    return xyz_quat


class Collision:
    def __init__(self, task: BaseTask):
        self.pin_mod, self.col_mod = create_model(**extract_from_task(task))
        self.task = task
        self.robot_pose = pose_as_matrix_to_pose_as_quat(task.get_robot_pose())

    def is_config_valid(self, configuration: Configuration):

        config = np.array(config[:-len(self.task.robot.initial_configuration())] + list(self.robot_pose) + \
                 config[-len(self.task.robot.initial_configuration()):])

        # Create data structures
        data = self.pin_mod.createData()
        geom_data = pin.GeometryData(self.col_mod)

        # Compute all the collisions
        print(pin.computeCollisions(self.pin_mod, data, self.col_mod, geom_data, config, False))

        for k in range(len(self.col_mod.collisionPairs)):
            cr = geom_data.collisionResults[k]
            cp = self.col_mod.collisionPairs[k]
            print("collision pair:", cp.first, ",", cp.second, "- collision:", "Yes" if cr.isCollision() else "No")

    def vizulize_through_pinocchio(self, configuration: Configuration):

        config = configuration.to_numpy()

        config = np.array(config[:-len(self.task.robot.initial_configuration())] + list(self.robot_pose) + \
                          config[-len(self.task.robot.initial_configuration()):])

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


task = ShelfTask(demo_id=2, robot=PandaRobot(), robot_pose_id=1)

collision = Collision(task)

config = [0.4, -0.3, 0, 0, 0, 0, 1] + [-0.4, 0.3, 0, 0, 0, 0, 1] + [0.4, 0.3, 0, 0, 0, 0, 1] + \
         [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4, 0., 0.]

collision.is_config_valid(config)
collision.vizulize_through_pinocchio(config)
pass
