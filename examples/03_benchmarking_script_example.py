from guided_tamp_benchmark.scripts.benchmark import Benchmark
from guided_tamp_benchmark.tasks import ShelfTask
from guided_tamp_benchmark.models.robots import *

import time

import numpy as np

b = Benchmark()
corba_server = CorbaServer(models_package=get_models_data_directory())

task = ShelfTask(1, PandaRobot(), 2)


b.do_benchmark(task=task, planner=MultiContactPlanner, seeds=[1],
               planner_arg={"handles_mode": 'exclude_side', "max_iter": 100000, "use_euclidean_distance": True,
                            "optimize_path_iter": 100, "verbose": True}, max_planning_time=60, delta=0.001)
