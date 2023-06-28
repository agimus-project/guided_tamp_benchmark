#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 04.04.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>
from __future__ import annotations

import pathlib
import time
import dill
import numpy as np

from guided_tamp_benchmark.core import BasePlanner
from guided_tamp_benchmark.benchmark import BenchmarkResult
from guided_tamp_benchmark.tasks import BaseTask
from collections import defaultdict


class Benchmark:
    """Benchmark class store results internally in nested dictionaries in a following
     structure:
       planer_name -> task_name -> demo_id -> robot_name -> robot_pose_id -> seed_id
    To get results for you planner for panda in shelf1 task do:
        res = benchmark['your_planner_name']['shelf1'][0]['panda'][0][0]
    The 'res' variable is instance of class BenchmarkResult:
    """

    def __init__(self):
        self.results = defaultdict(
            lambda: defaultdict(
                lambda: defaultdict(
                    lambda: defaultdict(
                        lambda: defaultdict(lambda: defaultdict(BenchmarkResult))
                    )
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
            benchmark_result: BenchmarkResult = self.results[p.name][task.task_name][
                task.demo.demo_id
            ][task.robot.name][task.demo.pose_id][s]
            benchmark_result.is_solved = res

            if not benchmark_result.is_solved:
                p.reset()
                continue

            path = p.get_path()
            path_as_config = [
                path.interpolate(t) for t in np.arange(0, 1 + delta, delta)
            ]
            p.reset()

            benchmark_result.computation_time = end_solve_t - start_solve_t
            benchmark_result.path_len = task.compute_lengths(path_as_config)
            benchmark_result.subsampled_path = path_as_config
            benchmark_result.number_of_grasps = task.compute_n_grasps(path_as_config)

    def save_benchmark(self, results_path: str | pathlib.Path):
        """saves the benchmarking results to the given file"""
        dill.dump(self.results, open(results_path, "wb"))

    def load_benchmark(self, results_path: str | pathlib.Path):
        """Load the benchmarking results from a given file. The results are stored
        internally."""
        self.results = dill.load(open(results_path, "rb"))
