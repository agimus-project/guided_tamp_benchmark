#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-10-13
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from abc import abstractmethod
from typing import List


class BaseObject(object):

    @abstractmethod
    def initial_configuration(self) -> List[float]:
        """Return initial configuration of the object."""
        pass

    @classmethod
    @abstractmethod
    def handles(cls, prefix: str = "") -> List[str]:
        """Returns the list of all handle names defined by the object with optional :param prefix. """
        pass

    @classmethod
    @abstractmethod
    def contact_surfaces(cls, prefix: str = "") -> List[str]:
        """Returns the list of all contact surface names defined by the object with optional :param prefix. """
        pass
