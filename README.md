# Brainbot Py
### Author: Thomas Herring

Brainbot Py is a Python library that provides an interface for running OMPL motion planning inside known volumes on simulated robots in PyBullet through Grapeshot. This library aims to explore methods for efficient motion planning with costs for volume-reducing constraints on robot kinematics.

## Features

- Easy integration with PyBullet for robot simulation
- Utilizes OMPL for motion planning
- Custom samplers for primitive workspace geometries
- Deterministic robot C-Space samplers for workspace pointclouds

## Installation

To install Brainbot Py as an editable library, simply run the following command:

```bash
pip install -e .
