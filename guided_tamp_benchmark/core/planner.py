#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 04.04.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from guided_tamp_benchmark.tasks import BaseTask
    from guided_tamp_benchmark.core import Path

from abc import ABC, abstractmethod


class BasePlanner(ABC):
    """
    Abstract base class for all planners.
    To create a new planner class, please implement all the abstract functions and
    properties.
    """

    @abstractmethod
    def __init__(
        self, task: BaseTask, max_planning_time: float, random_seed: int, **kwargs
    ):
        self.task = task
        self.max_planning_time = max_planning_time
        self.random_seed = random_seed

    @property
    @abstractmethod
    def name(self) -> str:
        """returns the name of the planner as a string"""
        pass

    @abstractmethod
    def solve(self) -> bool:
        """solves the given task, returns bool indicating whether solve was successful
        or not"""
        pass

    @abstractmethod
    def get_path(self) -> Path:
        """returns solution as Path class function"""
        pass

    def reset(self):
        """Reset internal state of the planner. Called in the benchmark after each
        planning."""
        pass
