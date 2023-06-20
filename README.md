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
There are examples of usage in directory examples. Here are some additional examples.
### Task class
There are three tasks, Shelf Task, Waiter Task, Tunnel Task.
Here is an example of task initialization.

```code
from guided_tamp_benchmark.tasks import ShelfTask
from guided_tamp_benchmark.models.robots import *


task = ShelfTask(0, PandaRobot(), 1)
```

In this example, Shelf Task with Panda Robot at pose num. 1. with one object was initialized.

Description of each task follows.


### Shelf Task
The shelf task is composed of a table, a shelf and a varying
set of objects that the robotic manipulator is supposed to
arrange, i.e. to move the objects to the predefined poses
on the table or on the shelf. The complexity of the task
is controlled by the number of objects that the robot should
arrange. This task is challenging for state-of-the-art planners
because it requires moving multiple objects in a single
planning task.
![Logs](images/shelf_task.png)
### Tunnel Task
The tunnel task consists of a tunnel and a single object that
should be transferred through the tunnel. The tunnel is thin
enough so that the robot can place an object inside the tunnel
on one side and pick it up from the other side. The challenge
lies in the narrow passage in the admissible configuration
space that needs to be discovered by the planner.
![Logs](images/tunnel_task.png)
### Waiter Task
The waiter task simulates the job of waiter, in which a set of
objects needs to be transferred from one location to another
distant location. Waiters use a tray to minimize the walked
distance. In our simulation, a mobile robot is equipped with
a tray-like space that it can use for transferring objects.
Discovering the tray in the planning state-space is non-trivial,
which makes this task challenging for the planners that do
not utilize demonstrations
![Logs](images/waiter_task.png)