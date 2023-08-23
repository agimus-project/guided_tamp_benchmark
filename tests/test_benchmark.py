#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-06-28
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
from pathlib import Path

from guided_tamp_benchmark.benchmark import Benchmark, BenchmarkResult


class TestBenchmarkScript(unittest.TestCase):
    def test_saving(self):
        path = Path("/tmp/results.pkl")
        b = Benchmark(path)
        b.save_benchmark()
        self.assertTrue(path.exists())

    def test_save_load(self):
        path = Path("/tmp/results1.pkl")
        b = Benchmark(path)
        b.results["planner"]["shelf1"][0]["panda"][0][0] = BenchmarkResult(
            True, 10.0, (1, 2, 3.0), list(), 10
        )
        b.save_benchmark()
        self.assertTrue(path.exists())
        del b
        b = Benchmark(path)
        b.load_benchmark()
        self.assertTrue("planner" in b.results)
        self.assertTrue("shelf1" in b.results["planner"])
        res: BenchmarkResult = b.results["planner"]["shelf1"][0]["panda"][0][0]
        self.assertTrue(res.is_solved)
        self.assertAlmostEqual(res.computation_time, 10.0)
        self.assertAlmostEqual(res.path_len[0], 1)
        self.assertAlmostEqual(res.path_len[1], 2)
        self.assertAlmostEqual(res.path_len[2], 3)
        self.assertEqual(res.subsampled_path, list())
        self.assertEqual(res.number_of_grasps, 10)


if __name__ == "__main__":
    unittest.main()
