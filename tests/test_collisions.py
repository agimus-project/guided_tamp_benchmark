import unittest
import pickle

from guided_tamp_benchmark.core import *
from guided_tamp_benchmark.tasks import *
from guided_tamp_benchmark.models.robots import *
import pathlib
from typing import Tuple, List


def collision_check_cycle(task: BaseTask, configs: List[Configuration]) -> Tuple[bool, int]:
    for i, c in enumerate(configs):
        if not task.collision.is_config_valid(c):
            return True, i

    return False, -1


def grasp_check_cycle(task: BaseTask, configs: List[Configuration], graps: list) -> bool:
    for i, c in enumerate(configs):
        if not task._check_grasp_constraint(c, delta=0.0001) == graps[i]:
            return False

    return True


def placement_check_cycle(task: BaseTask, configs: List[Configuration], placement: list) -> bool:
    for i, c in enumerate(configs):
        if not task._check_place_constraint(c) == placement[i]:
            return False

    return True


class CollisionFunctionsTestCase(unittest.TestCase):
    def test_collision_checking(self):
        # checks whether the collision checking gives out the same results as previously
        path = pathlib.Path(__file__).parent.joinpath('test_configs.pkl')
        data = pickle.load(open(path, 'rb'))
        self.assertEqual(
            collision_check_cycle(WaiterTask(0, KukaMobileIIWARobot(), 1), data["waiter_0_kmr_0"]['configs']),
            (False, -1))
        self.assertEqual(
            collision_check_cycle(TunnelTask(0, UR5Robot(), 1), data["tunnel_0_ur_1"]['configs']), (False, -1))
        self.assertEqual(
            collision_check_cycle(ShelfTask(1, PandaRobot(), 1), data["shelf_1_panda_1"]['configs']), (False, -1))

    def test_grasp_checking(self):
        # checks whether the grasp checking gives out the same results as previously
        path = pathlib.Path(__file__).parent.joinpath('test_configs.pkl')
        data = pickle.load(open(path, 'rb'))
        self.assertEqual(grasp_check_cycle(WaiterTask(0, KukaMobileIIWARobot(), 1), data["waiter_0_kmr_0"]['configs'],
                                           data["waiter_0_kmr_0"]['grasps']), True)
        self.assertEqual(grasp_check_cycle(TunnelTask(0, UR5Robot(), 1), data["tunnel_0_ur_1"]['configs'],
                                           data["tunnel_0_ur_1"]['grasps']), True)
        self.assertEqual(grasp_check_cycle(ShelfTask(1, PandaRobot(), 1), data["shelf_1_panda_1"]['configs'],
                                           data["shelf_1_panda_1"]['grasps']), True)

    def test_placement_checking(self):
        # checks whether the placement checking gives out the same results as previously
        path = pathlib.Path(__file__).parent.joinpath('test_configs.pkl')
        data = pickle.load(open(path, 'rb'))
        self.assertEqual(
            placement_check_cycle(WaiterTask(0, KukaMobileIIWARobot(), 1), data["waiter_0_kmr_0"]['configs'],
                                  data["waiter_0_kmr_0"]['placements']), True)
        self.assertEqual(placement_check_cycle(TunnelTask(0, UR5Robot(), 1), data["tunnel_0_ur_1"]['configs'],
                                               data["tunnel_0_ur_1"]['placements']), True)
        self.assertEqual(placement_check_cycle(ShelfTask(1, PandaRobot(), 1), data["shelf_1_panda_1"]['configs'],
                                               data["shelf_1_panda_1"]['placements']), True)


if __name__ == '__main__':
    unittest.main()
