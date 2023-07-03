#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-06-29
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from pathlib import Path
from imageio import imwrite

from guided_tamp_benchmark.core import Configuration
from guided_tamp_benchmark.models.robots import *
from guided_tamp_benchmark.tasks import *
from guided_tamp_benchmark.tasks.demonstration import Demonstration
from guided_tamp_benchmark.tasks.renderer import Renderer

r = None

for task_cls, task_name in [(TunnelTask, "tunnel"), (ShelfTask, "shelf")]:
    for robot_cls in [PandaRobot, UR5Robot, KukaIIWARobot]:
        output_folder = Path(f"/tmp/robot_poses/{task_cls.__name__}")
        output_folder.mkdir(parents=True, exist_ok=True)

        for demo_id in Demonstration.get_demonstration_ids(task_name):
            nposes = Demonstration.get_number_of_robot_poses(
                task_name, demo_id, robot_cls.name
            )
            for robot_pose_id in range(0, nposes):
                try:
                    task = task_cls(
                        demo_id=demo_id, robot=robot_cls(), robot_pose_id=robot_pose_id
                    )
                except FileNotFoundError as e:
                    print(e)
                    continue
                print(f"DemoId: {demo_id}, PoseId: {robot_pose_id}")
                if r is not None:
                    r.scene.clear()
                r = Renderer(task=task, scene=r.scene if r is not None else None)
                r.render_configuration(
                    Configuration(
                        task.robot.initial_configuration(),
                        task.demo.subgoal_objects_poses[:, 0],
                    )
                )
                img = r.scene.render_image()
                filename = (
                    f"{task_cls.__name__}_{robot_cls.__name__}"
                    f"_{demo_id}_{robot_pose_id}.png"
                )
                imwrite(output_folder.joinpath(filename), img)
