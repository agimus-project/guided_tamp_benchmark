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

import xml.etree.ElementTree as ET

from guided_tamp_benchmark.models import parse_contacts_grippers_handles


class BaseObject(object):
    rootJointType = "freeflyer"
    urdfSuffix = ""
    srdfSuffix = ""
    name = "base"

    def __init__(self, create_srdf_file=True):
        self.create_srdf_file = create_srdf_file
        self.fd_urdf, self.urdfFilename = tempfile.mkstemp(suffix=".urdf", text=True)
        if self.create_srdf_file:
            self.fd_srdf, self.srdfFilename = tempfile.mkstemp(
                suffix=".srdf", text=True
            )

    @abstractmethod
    def initial_configuration(self) -> List[float]:
        """Return initial configuration of the object."""
        pass

    @classmethod
    @abstractmethod
    def handles(cls, prefix: str = "") -> List[str]:
        """Returns the list of all handle names defined by the object with optional
        :param prefix."""
        pass

    @classmethod
    @abstractmethod
    def contact_surfaces(cls, prefix: str = "") -> List[str]:
        """Returns the list of all contact surface names defined by the object with
        optional :param prefix."""
        pass

    def __del__(self):
        os.unlink(self.urdfFilename)
        if self.create_srdf_file:
            os.unlink(self.srdfFilename)

    def get_contacts_info(self) -> dict:
        """returns contacts in a dictionary of a form contacts["name"] =
        {"link": str, "shapes": np.array}"""
        tree = ET.parse(self.srdfFilename)
        root = tree.getroot()
        contacts, _, _ = parse_contacts_grippers_handles(
            root, contacts=True, grippers=False, handles=False
        )
        return contacts

    def get_handles_info(self) -> dict:
        """returns handles in a dictionary of a form handles["name"] =
        {"link": str, "pose": list,
         "clearance"" float}"""
        tree = ET.parse(self.srdfFilename)
        root = tree.getroot()
        _, _, handles = parse_contacts_grippers_handles(
            root, contacts=False, grippers=False, handles=True
        )
        return handles
