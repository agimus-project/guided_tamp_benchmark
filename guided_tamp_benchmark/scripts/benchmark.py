#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 04.04.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>
import time

from guided_tamp_benchmark.core import BasePlanner
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
            max_planning_time: float = 60,
    ):
        for s in seeds:
            try:
                p = planner(
                    task=task,
                    max_planning_time=max_planning_time,
                    seeds=s,
                    **planner_arg
                )
            except Exception as e:
                print(e)
                continue
            start_solve_t = time.time()
            try:
                res = p.solve()
            except Exception as e:
                print("ERROR")
                print(e)
                res = False
            p.solve()
            end_solve_t = time.time()

            print(
                f"{task.robot.name} robot pose {task.demo.pose_id},"
                f" seed {s}, solved: {res}")
            self.results[planner.name][task.task_name][task.demo.demo_id][
                task.robot.name][task.demo.pose_id][s][
                "is_solved"] = res

            if res:
                self.results[planner.name][task.task_name][task.demo.demo_id][
                    task.robot.name][task.demo.pose_id][s][
                    "time"] = (end_solve_t - start_solve_t)

                self.results[planner.name][task.task_name][task.demo.demo_id][
                    task.robot.name][task.demo.pose_id][s][
                    "path_len"] = task.compute_lengths(
                    planner.get_path_as_configurations())

                self.results[planner.name][task.task_name][task.demo.demo_id][
                    task.robot.name][task.demo.pose_id][s][
                    "configs"] = planner.get_path_as_configurations()

                self.results[planner.name][task.task_name][task.demo.demo_id][
                    task.robot.name][task.demo.pose_id][s][
                    "grasp_number"] = task.compute_n_grasps(
                    planner.get_path_as_configurations())

            else:
                continue

    def save_benchmark(self, results_path: str):
        pass


if __name__ == "__main__":
    pass
