#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-06-28
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
from pathlib import Path

from guided_tamp_benchmark.scripts.benchmark import Benchmark


class TestBenchmarkScript(unittest.TestCase):
    def test_saving(self):
        path = Path("/tmp/results.pkl")
        b = Benchmark()
        b.save_benchmark(path)
        self.assertTrue(path.exists())


if __name__ == "__main__":
    unittest.main()
