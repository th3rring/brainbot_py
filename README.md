# Brainbot Py
### Author: Thomas Herring

Brainbot Py is a Python library that provides an interface for running OMPL motion planning inside known volumes on simulated robots in PyBullet through Grapeshot. This library aims to explore methods for efficient motion planning with costs for volume-reducing constraints on robot kinematics.

## Features

- Integration with PyBullet - through Grapeshot - for robot simulation
- Utilizes OMPL for motion planning for a UR5 robot
- Analytical inverse and forward kinematics solver
- Position samplers for primitive workspace geometries
- Deterministic robot C-Space samplers for workspace regions

## Installation

To install Brainbot Py as an editable developer library, simply run the following command:

```bash
pip install -e .
