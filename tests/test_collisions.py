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
from typing import Tuple, List


def collision_check_cycle(
        task: BaseTask, configs: List[Configuration]
) -> Tuple[bool, int]:
    for i, c in enumerate(configs):
        if not task.collision.is_config_valid(c):
            return True, i

    return False, -1


def grasp_check_cycle(
        task: BaseTask, configs: List[Configuration], graps: list
) -> bool:
    for i, c in enumerate(configs):
        if not task._check_grasp_constraint(c, delta=0.0001) == graps[i]:
            return False

    return True


def placement_check_cycle(
        task: BaseTask, configs: List[Configuration], placement: list
) -> bool:
    for i, c in enumerate(configs):
        if not task._check_place_constraint(c) == placement[i]:
            return False

    return True


class CollisionFunctionsTestCase(unittest.TestCase):
    def test_collision_checking(self):
        # checks whether the collision checking gives out the same results as previously
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
                TunnelTask(0, UR5Robot(), 1), data["tunnel_0_ur_1"]["configs"]
            ),
            (False, -1),
        )
        self.assertEqual(
            collision_check_cycle(
                ShelfTask(1, PandaRobot(), 1), data["shelf_1_panda_1"]["configs"]
            ),
            (False, -1),
        )

    def test_grasp_checking(self):
        # checks whether the grasp checking gives out the same results as previously
        path = pathlib.Path(__file__).parent.joinpath("test_configs.pkl")
        data = pickle.load(open(path, "rb"))
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
                TunnelTask(0, UR5Robot(), 1),
                data["tunnel_0_ur_1"]["configs"],
                data["tunnel_0_ur_1"]["grasps"],
            ),
            True,
        )
        self.assertEqual(
            grasp_check_cycle(
                ShelfTask(1, PandaRobot(), 1),
                data["shelf_1_panda_1"]["configs"],
                data["shelf_1_panda_1"]["grasps"],
            ),
            True,
        )

    def test_grasp_counting(self):
        # checks whether the grasp counting gives out the same results as previously
        path = pathlib.Path(__file__).parent.joinpath("test_configs.pkl")
        data = pickle.load(open(path, "rb"))
        self.assertEqual(
            ShelfTask(1, PandaRobot(), 1).compute_n_grasps(
                data["shelf_1_panda_1"]["configs"]
            ),
            2,
        )
        self.assertEqual(
            TunnelTask(0, UR5Robot(), 1).compute_n_grasps(
                data["tunnel_0_ur_1"]["configs"]
            ),
            3,
        )

    def test_placement_checking(self):
        # checks whether the placement checking gives out the same results as previously
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
                TunnelTask(0, UR5Robot(), 1),
                data["tunnel_0_ur_1"]["configs"],
                data["tunnel_0_ur_1"]["placements"],
            ),
            True,
        )
        self.assertEqual(
            placement_check_cycle(
                ShelfTask(1, PandaRobot(), 1),
                data["shelf_1_panda_1"]["configs"],
                data["shelf_1_panda_1"]["placements"],
            ),
            True,
        )

    def test_valid_path(self):
        # test path_is_successful function
        path = pathlib.Path(__file__).parent.joinpath("test_configs.pkl")
        data = pickle.load(open(path, "rb"))

        res = WaiterTask(0, KukaMobileIIWARobot(), 1).path_is_successful(
            data["waiter_0_kmr_0"]["configs"])
        self.assertEqual(res, (False,
                               "last configuration of robot kmr_iiwa doesn't"
                               " match with its goal configuration"),
                         msg=res[1] + " tunnel task test")

        res = TunnelTask(0, UR5Robot(), 1).path_is_successful(
            data["tunnel_0_ur_1"]["configs"])
        self.assertEqual(res[0], True, msg=res[1] + " tunnel task test")

        res = ShelfTask(1, PandaRobot(), 1).path_is_successful(
            data["shelf_1_panda_1"]["configs"])
        self.assertEqual(res, (False,
                               "last pose of object obj_000002 doesn't match with its "
                               "goal configuration from demonstration"),
                         msg=res[1] + " shelf task 2 test")

        st = ShelfTask(2, PandaRobot(), 1)
        res = st.path_is_successful([Configuration(st.robot.initial_configuration(),
                                                   st.demo.subgoal_objects_poses[:, 0]),
                                     Configuration(st.robot.initial_configuration(),
                                                   st.demo.subgoal_objects_poses[:,
                                                   -1])])
        self.assertEqual(res, (False, 'object obj_000002, moved between configuration '
                                      '1 and 0, even though its under placement'
                                      ' constraint.'),
                         msg=res[1] + " shelf task 3 test")

        if __name__ == "__main__":
            unittest.main()
