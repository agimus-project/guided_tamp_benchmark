#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 13.03.23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz> and
#               David Kovar <kovarda8@fel.cvut.cz>

from abc import ABC, abstractmethod

from guided_tamp_benchmark.core import Configuration


class Path(ABC):
    @abstractmethod
    def interpolate(self, t: float) -> Configuration:
        """Compute the configuration at interpolated time like value t
        Args:
            t: normalized interpolation time from 0 to 1
        """
