# GuidedTAMPBenchmark

## Overview
This is an overview for GuidedTAMPBenchmark repository. 
This repo is part of paper submission for robotics ICRA conference 2023. 
This repository was used for testing and benchmarking of our submission.
### Core
In `guided_tamp_benchmark.core` is description of base planner function that is compatible with benchmarking script and is to be used as template.
Description of configuration class, used for storing scene configuration, and path class, which is a template class for storing path planned by planner is also there.
### Models
In `guided_tamp_benchmark.models` is description of robots, furniture and objects used in different tasks of benchmark.
### Scripts
In `guided_tamp_benchmark.scripts` is class benchmark that can be used for running benchmark on planner.
### Tasks
In `guided_tamp_benchmark.tasks` are classes with descriptions of benchmarking tasks, renderer, demonstration class used for storing and loading demonstration files saved as .pkl files and collision class used for collision checking for path evaluation.

## Installation
```bash
conda create -n gtamp python=3.10 poetry
conda activate gtamp
poetry install
```

## Usage
There are examples of usage in directory examples.
### Task class
There are three tasks, Shelf Task, Waiter Task, Tunnel Task.
Here is an example of task initialization.

```code
from guided_tamp_benchmark.tasks import ShelfTask
from guided_tamp_benchmark.models.robots import *


task = ShelfTask(0, PandaRobot(), 1)
```

In this example, Shelf Task with Panda Robot at pose num. 1. with one object was initialized.