import unittest
import pickle

from guided_tamp_benchmark.core import Configuration
from guided_tamp_benchmark.tasks import BaseTask, ShelfTask, TunnelTask, WaiterTask
from guided_tamp_benchmark.models.robots import (
    PandaRobot,
    UR5Robot,
    KukaMobileIIWARobot,
)
import pathlib


def collision_check_cycle(
    task: BaseTask, configs: list[Configuration]
) -> tuple[bool, int]:
    for i, c in enumerate(configs):
        if not task.collision.is_config_valid(c):
            return True, i

    return False, -1


def grasp_check_cycle(
    task: BaseTask, configs: list[Configuration], graps: list
) -> bool:
    for i, c in enumerate(configs):
        if task._check_grasp_constraint(c, delta=0.0001) == graps[i]:
            return True

    return False


def placement_check_cycle(
    task: BaseTask, configs: list[Configuration], placement: list
) -> bool:
    for i, c in enumerate(configs):
        if not task._check_place_constraint(c) == placement[i]:
            return False

    return True


class CollisionFunctionsTestCase(unittest.TestCase):
    def test_collision_checking(self):
        """
        checks whether the collision checking gives out the same results as previously
        """
        path = pathlib.Path(__file__).parent.joinpath("test_configs.pkl")
        data = pickle.load(open(path, "rb"))
        self.assertEqual(
            collision_check_cycle(
                WaiterTask(0, KukaMobileIIWARobot(), 1),
                data["waiter_0_kmr_0"]["configs"],
            ),
            (False, -1),
        )
        self.assertEqual(
            collision_check_cycle(
                TunnelTask(0, PandaRobot(), 0), data["tunnel_0_panda_0"]["configs"]
            ),
            (False, -1),
        )
        self.assertEqual(
            collision_check_cycle(
                ShelfTask(1, UR5Robot(), 3), data["shelf_1_ur_3"]["configs"]
            ),
            (False, -1),
        )

    def test_grasp_checking(self):
        """checks whether the grasp checking gives out the same results as previously"""
        path = pathlib.Path(__file__).parent.joinpath("test_configs.pkl")
        data = pickle.load(open(path, "rb"))
        task = WaiterTask(0, KukaMobileIIWARobot(), 1)

        self.assertEqual(
            grasp_check_cycle(
                WaiterTask(0, KukaMobileIIWARobot(), 1),
                data["waiter_0_kmr_0"]["configs"],
                data["waiter_0_kmr_0"]["grasps"],
            ),
            True,
        )
        self.assertEqual(
            grasp_check_cycle(
                TunnelTask(0, PandaRobot(), 0),
                data["tunnel_0_panda_0"]["configs"],
                data["tunnel_0_panda_0"]["grasps"],
            ),
            True,
        )
        self.assertEqual(
            grasp_check_cycle(
                ShelfTask(1, UR5Robot(), 3),
                data["shelf_1_ur_3"]["configs"],
                data["shelf_1_ur_3"]["grasps"],
            ),
            True,
        )

    def test_grasp_counting(self):
        """checks whether the grasp counting gives out the same results as previously"""
        path = pathlib.Path(__file__).parent.joinpath("test_configs.pkl")
        data = pickle.load(open(path, "rb"))
        self.assertEqual(
            WaiterTask(0, KukaMobileIIWARobot(), 0).compute_n_grasps(
                data["waiter_0_kmr_0"]["configs"]
            ),
            data["waiter_0_kmr_0"]["n_grasps"],
        )
        self.assertEqual(
            ShelfTask(1, UR5Robot(), 3).compute_n_grasps(
                data["shelf_1_ur_3"]["configs"]
            ),
            data["shelf_1_ur_3"]["n_grasps"],
        )
        self.assertEqual(
            TunnelTask(0, PandaRobot(), 0).compute_n_grasps(
                data["tunnel_0_panda_0"]["configs"]
            ),
            data["tunnel_0_panda_0"]["n_grasps"],
        )

    def test_placement_checking(self):
        """
        checks whether the placement checking gives out the same results as previously
        """
        path = pathlib.Path(__file__).parent.joinpath("test_configs.pkl")
        data = pickle.load(open(path, "rb"))
        self.assertEqual(
            placement_check_cycle(
                WaiterTask(0, KukaMobileIIWARobot(), 1),
                data["waiter_0_kmr_0"]["configs"],
                data["waiter_0_kmr_0"]["placements"],
            ),
            True,
        )
        self.assertEqual(
            placement_check_cycle(
                TunnelTask(0, PandaRobot(), 0),
                data["tunnel_0_panda_0"]["configs"],
                data["tunnel_0_panda_0"]["placements"],
            ),
            True,
        )
        self.assertEqual(
            placement_check_cycle(
                ShelfTask(1, UR5Robot(), 3),
                data["shelf_1_ur_3"]["configs"],
                data["shelf_1_ur_3"]["placements"],
            ),
            True,
        )


if __name__ == "__main__":
    unittest.main()
