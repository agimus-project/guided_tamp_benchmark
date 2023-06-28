#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-06-28
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
from dataclasses import dataclass

from guided_tamp_benchmark.core import Configuration


@dataclass
class BenchmarkResult:
    """Benchmark result computed by a single planner for a single planning instance.
    Attributes:
        is_solved: indicates if the solution was found by a given time_limit
        computation_time: time it takes to find a solution
        path_len: computed lengths of the path @see BaseTask.compute_lengths
        subsampled_path: list of configurations from a computed path
        number_of_grasps: how many times the objects were grasped by a gripper
    """

    is_solved: bool = False
    computation_time: float | None = None
    path_len: tuple[float, float, float] | None = None
    subsampled_path: list[Configuration] | None = None
    number_of_grasps: int | None = None
