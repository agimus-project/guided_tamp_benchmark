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
    """
    Benchmark class store results internally in nested dictionaries in a following
     structure:
       planer_name -> task_name -> demo_id -> robot_name -> robot_pose_id -> seed_id
    To get results for you planner for panda in shelf1 task do:
        res = benchmark['your_planner_name']['shelf1'][0]['panda'][0][0]
    The 'res' variable is instance of class BenchmarkResult:
    """

    def __init__(self, results_path: str | pathlib.Path):
        self.results_path = results_path
        if pathlib.Path(results_path).exists():
            self.load_benchmark()
        else:
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
        obj_update_pose_tuples: list = [],
        furn_id_to_update: int | None = None,
        furniture_update_pose: np.array = np.eye(4)
    ):
        """runs benchmarking for the given planner with given arguments, on given task
        and seeds with the specified max planning time. Results will be saved in
        Benchmark.results dictionary. Argument delta is a step by which the path
        returned by planner will be interpolated."""

        for s in seeds:
            res = None
            while res is None:
                try:
                    p = planner(
                        task=task,
                        max_planning_time=max_planning_time,
                        random_seed=s,
                        **planner_arg,
                    )
                    task = p.task
                    if furn_id_to_update is not None:
                        p.update_object_to_match_furniture_poses(
                            furniture_update_pose, furn_id_to_update)

                    if len(obj_update_pose_tuples) > 0:
                        for time_id, obj_id, pose in obj_update_pose_tuples:
                            _ = p.update_object_poses_matching_time_id(
                                pose_update=pose, obj_id=obj_id,
                                time_id=time_id)


                    start_solve_t = time.time()
                    res = p.solve()
                    if not res:
                        p.reset()
                except Exception as e:
                    print(f"ERROR {e}")
                    import traceback
                    print(traceback.format_exc())
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
            try:
                path = p.get_path()
                path_as_config = [
                    path.interpolate(t) for t in np.arange(0, 1 + delta, delta)
                ]
            except Exception as e:
                print('encountered path error')
                path_as_config = []
                for t in np.arange(0, 1 + delta, delta):
                    try:
                        q = path.interpolate(t)
                        path_as_config.append(q)
                    except:
                        path_as_config.append(path_as_config[-1])                
            p.reset()

            benchmark_result.computation_time = end_solve_t - start_solve_t
            benchmark_result.path_len = task.compute_lengths(path_as_config)
            benchmark_result.subsampled_path = path_as_config
            benchmark_result.number_of_grasps = task.compute_n_grasps(path_as_config)

    def save_benchmark(self):
        """saves the benchmarking results to the given file"""
        with open(self.results_path, "wb") as f:
            dill.dump(self.results, f)

    def load_benchmark(self):
        """Load the benchmarking results from a given file. The results are stored
        internally."""
        with open(self.results_path, "rb") as f:
            self.results = dill.load(f)
