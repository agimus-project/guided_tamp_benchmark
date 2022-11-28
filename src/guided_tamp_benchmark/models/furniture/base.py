#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 13.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from abc import abstractmethod
from typing import List
import tempfile
import os


class FurnitureObject(object):
    rootJointType = "fix"
    urdfSuffix = ""
    srdfSuffix = ""
    name = ''

    def __init__(self) -> None:
        super().__init__()
        self.fd_urdf, self.urdfFilename = tempfile.mkstemp(suffix=".urdf", text=True)
        self.fd_srdf, self.srdfFilename = tempfile.mkstemp(suffix=".srdf", text=True)

    def __del__(self):
        os.unlink(self.urdfFilename)
        os.unlink(self.srdfFilename)

    @classmethod
    @abstractmethod
    def contact_surfaces(cls, prefix: str = "") -> List[str]:
        """Returns the list of all contact surface names defined by the object with optional :param prefix. """
        pass
