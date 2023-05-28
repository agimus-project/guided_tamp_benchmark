#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 04.04.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>
import time
import pickle
import numpy as np
import pinocchio as pin

from guided_tamp_benchmark.core import BasePlanner
from guided_tamp_benchmark.core.configuration import Configuration
from guided_tamp_benchmark.tasks import BaseTask
from collections import defaultdict


class Benchmark:
    results = defaultdict(
        lambda: defaultdict(
            lambda: defaultdict(
                lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))
            )
        )
    )

    def __init__(self):
        pass

    def do_benchmark(
            self,
            task: BaseTask,
            planner: BasePlanner,
            seeds,
            planner_arg: dict,
            q_start: Configuration,
            q_goal: Configuration,
            max_planning_time: float = 60
    ):
        """runs benchmarking for the given planner with given arguments, on given task
        and seeds with the specified max planning time. Results will be saved in
        Benchmark.results dictionary"""
        for s in seeds:
            try:
                p = planner(
                    task=task,
                    max_planning_time=max_planning_time,
                    random_seed=s,
                    **planner_arg
                )
            except Exception as e:
                print(e)
                continue

            start_solve_t = time.time()
            try:
                res = p.solve(q_start, q_goal)
            except Exception as e:
                print("ERROR")
                print(e)
                res = False
            end_solve_t = time.time()

            print(
                f"{task.robot.name} robot pose {task.demo.pose_id},"
                f" seed {s}, solved: {res}")
            self.results[p.name][task.task_name][task.demo.demo_id][
                task.robot.name][task.demo.pose_id][s][
                "is_solved"] = res

            if res:
                self.results[p.name][task.task_name][task.demo.demo_id][
                    task.robot.name][task.demo.pose_id][s][
                    "time"] = (end_solve_t - start_solve_t)

                self.results[p.name][task.task_name][task.demo.demo_id][
                    task.robot.name][task.demo.pose_id][s][
                    "path_len"] = task.compute_lengths(
                    p.get_path_as_configurations())

                self.results[p.name][task.task_name][task.demo.demo_id][
                    task.robot.name][task.demo.pose_id][s][
                    "configs"] = p.get_path_as_configurations()

                self.results[p.name][task.task_name][task.demo.demo_id][
                    task.robot.name][task.demo.pose_id][s][
                    "grasp_number"] = task.compute_n_grasps(
                    p.get_path_as_configurations())

            else:
                continue

    def save_benchmark(self, results_path: str):
        """saves the benchmarking results to the given file"""
        pickle.dump(self.results, open(results_path, "wb"))


if __name__ == "__main__":
    pass
