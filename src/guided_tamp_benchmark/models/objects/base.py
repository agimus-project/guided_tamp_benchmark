#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-10-13
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from abc import abstractmethod
from typing import List
import tempfile
import os

class BaseObject(object):
    rootJointType = "freeflyer"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, create_srdf_file=True):
        self.create_srdf_file = create_srdf_file
        self.fd_urdf, self.urdfFilename = tempfile.mkstemp(suffix=".urdf", text=True)
        if self.create_srdf_file:
            self.fd_srdf, self.srdfFilename = tempfile.mkstemp(suffix=".srdf", text=True)

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

    def __del__(self):
        os.unlink(self.urdfFilename)
        if self.create_srdf_file:
            os.unlink(self.srdfFilename)
