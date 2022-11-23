#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2022-11-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pathlib import Path


def get_models_data_directory() -> Path:
    """Get path to the data of the models."""
    return Path(__file__).resolve().parent.joinpath('data')


def get_robots_data_directory() -> Path:
    """Get path to the data of robots inside the models module"""
    return get_models_data_directory().joinpath('robots')


def get_ycbv_data_directory() -> Path:
    """Get path to the YCBV dataset data inside the models module"""
    return get_models_data_directory().joinpath('ycbv')
