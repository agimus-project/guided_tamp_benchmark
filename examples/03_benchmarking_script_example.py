from guided_tamp_benchmark.scripts.benchmark import Benchmark
from guided_tamp_benchmark.core import Configuration
from guided_tamp_benchmark.tasks import ShelfTask
from guided_tamp_benchmark.models.robots import *

import numpy as np

b = Benchmark()
corba_server = CorbaServer(models_package=get_models_data_directory())

task = ShelfTask(1, PandaRobot(), 2)

q_start = np.array(
    [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483, 0.0, 0.0,
     0.19688817569653652, 0.43598173447468297, 0.8579733385362113, -2.2077344199119482e-05,
     -5.2343860577041686e-05, -0.5494813008494817, 0.8355060124197029, 0.4736579365162766, 0.703866635494848,
     1.2970099689069035, -2.4345570964884216e-05, 0.00019926629002679053, 0.9972862589697721,
     -0.07362117474148033])

q_goal = np.array(
    [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483, 0.0, 0.0,
     0.1519456141147882, 0.7308948348874202, 1.307999330039207, 1.9700833926530784e-05, 1.810351376605795e-05,
     -0.6628614009803104, 0.7487421200751031, 0.5733565877626446, 0.140065775006681, 0.8470567797579087,
     -0.00011642206988091039, -4.866779495303524e-06, -0.04399296141741992, 0.9990318342114738])

c_start = Configuration.from_numpy(q_start, len(PandaRobot().initial_configuration()))
c_goal = Configuration.from_numpy(q_start, len(PandaRobot().initial_configuration()))

b.do_benchmark(task=task, planner=MultiContactPlanner, seeds=[1],
               planner_arg={"handles_mode": 'exclude_side', "max_iter": 100000, "use_euclidean_distance": True,
                            "optimize_path_iter": 100, "verbose": True}, max_planning_time=60, q_start=c_start,
               q_goal=c_goal, delta=0.001)

corba_server.reset_problem()
time.sleep(5)
