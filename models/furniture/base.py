#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 13.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from abc import abstractmethod
from typing import List


class BaseObject(object):

    @classmethod
    @abstractmethod
    def contact_surfaces(cls, prefix: str = "") -> List[str]:
        """Returns the list of all contact surface names defined by the object with optional :param prefix. """
        pass
