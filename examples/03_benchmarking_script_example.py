from guided_tamp_benchmark.core import Path
from guided_tamp_benchmark.benchmark import Benchmark
from guided_tamp_benchmark.tasks import ShelfTask, BaseTask
from guided_tamp_benchmark.models.robots import *
from guided_tamp_benchmark.core.planner import BasePlanner


class DummyPlanner(BasePlanner):
    def __init__(
        self, task: BaseTask, max_planning_time: float, random_seed: int, **kwargs
    ):
        super(DummyPlanner, self).__init__(
            task, max_planning_time, random_seed, **kwargs
        )

    @property
    def name(self) -> str:
        return "DummyPlanner"

    def solve(self) -> bool:
        raise NotImplementedError("You need to implement planner by yourself.")

    def get_path(self) -> Path:
        raise NotImplementedError("You need to implement construction of tha path")


task = ShelfTask(1, PandaRobot(), 2)
b = Benchmark()
b.do_benchmark(
    task=task,
    planner=DummyPlanner,
    seeds=[1],
    planner_arg={
        "handles_mode": "exclude_side",
        "max_iter": 100000,
        "use_euclidean_distance": True,
        "optimize_path_iter": 100,
        "verbose": True,
    },
    max_planning_time=60,
    delta=0.001,
)
