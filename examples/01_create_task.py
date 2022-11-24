import numpy as np
from guided_tamp_benchmark.models.robots import *
from guided_tamp_benchmark.tasks.demonstration import Demonstration
from guided_tamp_benchmark.tasks.base_task import BaseTask
from guided_tamp_benchmark.tasks.shelf_task import ShelfTask

T = 5
demo = Demonstration()
demo.task_name = 'test'
demo.demo_id = 0
demo.robot_name = 'panda'
demo.pose_id = 0
demo.object_ids = ['CUBOID_0.1_0.2_0.8', 'TRAY']
demo.objects_poses = np.eye(4).reshape(1, 1, 4, 4).repeat(T, axis=1).repeat(2, axis=0)
demo.contacts = np.zeros((2, T), dtype=bool)
demo.robot_pose = np.eye(4)
demo.furniture_ids = ['tunnel']
demo.furniture_poses = np.eye(4).reshape(1, 4, 4)
demo.furniture_params = [{'lengths': [0.1, 0.2, 0.3], 'tunnel_walls_thickness': 0.16, 'collision_walls_thickness': 0.1}]
demo.save(overwrite=True)

class TestTask(BaseTask):
    def __init__(self):
        super().__init__('test', 0, PandaRobot(), 0)
task = TestTask()