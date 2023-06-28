#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 04.04.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>
from __future__ import annotations
import time
import pickle
import numpy as np

from guided_tamp_benchmark.core import BasePlanner
from guided_tamp_benchmark.tasks import BaseTask
from collections import defaultdict


class Benchmark:
    def __init__(self):
        self.results = defaultdict(
            lambda: defaultdict(
                lambda: defaultdict(
                    lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))
                )
            )
        )

    def do_benchmark(
        self,
        task: BaseTask,
        planner: type[BasePlanner],
        seeds,
        planner_arg: dict,
        delta: float,
        max_planning_time: float = 60,
    ):
        """runs benchmarking for the given planner with given arguments, on given task
        and seeds with the specified max planning time. Results will be saved in
        Benchmark.results dictionary. Argument delta is a step by which the path
        returned by planner will be interpolated."""
        for s in seeds:
            try:
                p = planner(
                    task=task,
                    max_planning_time=max_planning_time,
                    random_seed=s,
                    **planner_arg,
                )
                start_solve_t = time.time()
                res = p.solve()
            except Exception as e:
                print(f"ERROR {e}")
                res = False
            end_solve_t = time.time()

            print(
                f"{task.robot.name} robot pose {task.demo.pose_id},"
                f" seed {s}, solved: {res}"
            )
            dict_entry = self.results[p.name][task.task_name][task.demo.demo_id][
                task.robot.name
            ][task.demo.pose_id][s]

            dict_entry["is_solved"] = res
            if not res:
                continue

            path = p.get_path()
            path_as_config = [
                path.interpolate(t) for t in np.arange(0, 1 + delta, delta)
            ]
            del p

            if res:
                dict_entry["time"] = end_solve_t - start_solve_t
                dict_entry["path_len"] = task.compute_lengths(path_as_config)
                dict_entry["configs"] = path_as_config
                dict_entry["grasp_number"] = task.compute_n_grasps(path_as_config)

    def save_benchmark(self, results_path: str):
        """saves the benchmarking results to the given file"""
        pickle.dump(self.results, open(results_path, "wb"))


if __name__ == "__main__":
    pass
