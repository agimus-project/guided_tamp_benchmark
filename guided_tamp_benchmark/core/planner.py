#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 04.04.23
#     Author: David Kovar <kovarda8@fel.cvut.cz>


from abc import ABC, abstractmethod

from guided_tamp_benchmark.core import Path


class BasePlanner(ABC):
    """
    Abstract base class for all planners.
    To create a new planner class, please implement all the abstract functions and
    properties.
    """

    @abstractmethod
    def __init__(self, task, max_planning_time: float, random_seed: int):
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @abstractmethod
    def solve(self) -> bool:
        pass

    @abstractmethod
    def get_path(self) -> Path:
        pass