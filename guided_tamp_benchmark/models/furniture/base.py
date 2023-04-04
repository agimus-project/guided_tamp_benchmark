#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 13.10.22
#     Author: David Kovar <kovarda8@fel.cvut.cz>

from abc import abstractmethod
from typing import List
import tempfile
import os

import xml.etree.ElementTree as ET

from guided_tamp_benchmark.models import parse_contacts_grippers_handles


class FurnitureObject(object):
    rootJointType = "fix"
    urdfSuffix = ""
    srdfSuffix = ""
    name = ""

    def __init__(self) -> None:
        super().__init__()
        self.fd_urdf, self.urdfFilename = tempfile.mkstemp(suffix=".urdf", text=True)
        self.fd_srdf, self.srdfFilename = tempfile.mkstemp(suffix=".srdf", text=True)

    def __del__(self):
        os.unlink(self.urdfFilename)
        os.unlink(self.srdfFilename)

    @abstractmethod
    def contact_surfaces(self, prefix: str = "") -> List[str]:
        """Returns the list of all contact surface names defined by the object with
        optional :param prefix."""
        pass

    def get_contacts_info(self) -> dict:
        """returns contacts in a dictionary of a form
        contacts["name"] = {"link": str, "shapes": np.array}"""
        tree = ET.parse(self.srdfFilename)
        root = tree.getroot()
        contacts, _, _ = parse_contacts_grippers_handles(
            root, contacts=True, grippers=False, handles=False
        )
        return contacts
