#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-24
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from typing import List, Union, Tuple

import numpy as np
import pinocchio as pin
from numpy.linalg import norm


class Configuration:
    """The configuration class consists of robot joint values and the list of poses of
    manipulated objects. It describes the whole configuration of the system."""

    def __init__(
        self, q: Union[List[float], np.array], poses: Union[List[np.array], np.array]
    ) -> None:
        """Initialize the configuration by the given robot configuration @param q and
        @param poses of objects either given by a list of N 4x4 transformations or by
        np.array of size Nx4x4, where N is the number of objects
        """
        super().__init__()
        self.q = np.asarray(q)
        self.poses = np.asarray(poses)

    @property
    def ndofs_robot(self):
        """Returns the number of degrees of freedom for the robot"""
        return self.q.shape[0]

    @property
    def nobjects(self):
        """Returns the number of objects"""
        return self.poses.shape[0]

    def to_numpy(self) -> np.array:
        """Return numpy array of the whole configuration in a form [robot_joints,
        obj1_pos, obj1_xyzw_quaternion, ...]"""
        return np.concatenate(
            [self.q] + [pin.SE3ToXYZQUAT(pin.SE3(pose)) for pose in self.poses]
        )

    @staticmethod
    def from_numpy(config: np.array, robot_ndofs: int):
        """Fill in the internal configuration from the configuration in a form shown in
        @method to_numpy"""
        config = config.copy()
        q = config[:robot_ndofs]
        nobjects = (config.shape[0] - robot_ndofs) // 7
        poses = [
            pin.XYZQUATToSE3(
                config[robot_ndofs + i * 7 : robot_ndofs + (i + 1) * 7]
            ).homogeneous
            for i in range(nobjects)
        ]
        return Configuration(q, poses)

    def distance(self, other) -> Tuple[float, float, float]:
        """Return 3 dimensional tuple containing distance of robot joint values, linear
        distance of objects, and rotational distance of objects."""
        dq = norm(self.q - other.q)
        dlin = norm(self.poses[:, :3, 3] - other.poses[:, :3, 3])
        drot = norm(
            [
                norm(pin.log3(p1[:3, :3].T @ p2[:3, :3]))
                for p1, p2 in zip(self.poses, other.poses)
            ]
        )
        return dq, dlin, drot
