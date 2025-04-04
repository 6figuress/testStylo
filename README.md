# TestPen Part
## Overview
The TestStylo repo is designed to control a robotic arm equipped with a pen gripper. The robot can pick up pens from a pen holder, switch between pens of different colors, and execute trajectories to draw or perform tasks. The project integrates kinematics, simulation, and hardware control to achieve precise movements.

## Features
Pen Holder Management: Pick up, switch, and drop pens of different colors.
Trajectory Execution: Generate and execute optimal trajectories for the robotic arm.
Simulation Support: Visualize and debug movements using PyBullet simulation.
Collision Detection: Ensure safe movements by avoiding collisions.
Multi-Pen Support: Handle multiple pens in a 2x4 grid configuration.

## Requirements
Python 3.11
Libraries:
numpy
pybullet
ur_ikfast
urbasic
simulator
## Installation

To install repo you can : 

uv add git+https://github.com/6figuress/testStylo.git

## Run 

uv run -m filename


## Key Components
PenHolder Class (in final.py)

To use the pen change in the pipeline just call the pen holder class and chaim the pick a pen function.
One example of pen change was always done in the final.py file the other two files were used for examples and to correctly place the color stand