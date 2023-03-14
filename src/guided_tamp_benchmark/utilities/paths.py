#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 13.03.23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz> and David Kovar <kovarda8@fel.cvut.cz>

import bisect
from abc import ABC, abstractmethod

import numpy as np
from numpy import linspace
from numpy.typing import ArrayLike

from guided_tamp_benchmark.tasks.configuration import Configuration

class Path(ABC):
    @abstractmethod
    def interpolate(self, t: float) -> Configuration:
        """Compute the configuration at interpolated time like value t
        Args:
            t: normalized interpolation time from 0 to 1
        """


class PathWaypoints(Path):
    """Picewise linear path"""

    def __init__(
        self, waypoints: list[Configuration], times: ArrayLike | None = None) -> None:
        super().__init__()
        assert len(waypoints) >= 2
        self.waypoints = waypoints
        self.times = times if times is not None else linspace(0, 1, len(self.waypoints))
        assert np.isclose(self.times[0], 0.0) and np.isclose(self.times[-1], 1)

    def interpolate(self, t: float) -> Configuration:
        assert 0 <= t <= 1, "Cannot extrapolate."
        ind = bisect.bisect_right(self.times, t, hi=len(self.waypoints) - 1)
        wa, wb = self.waypoints[ind - 1], self.waypoints[ind]
        ta, tb = self.times[ind - 1], self.times[ind]

        # TODO: figure out interpolation
        return wa.interpolate_toward(wb, t=(t - ta) / (tb - ta))