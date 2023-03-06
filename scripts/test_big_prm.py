import sys
import time
from pathlib import Path

# sys.path.insert(0, "/home/zak/src/ompl/py-bindings")
# sys.path.insert(0, "/home/zak/src/ompl/build/lib")

from fire import Fire

import numpy as np

from ompl import base as ob

import grapeshot as gs
import pybullet as pb

import ur5_ikfast

from brainbot.kinematics import solvers
from brainbot.plan.regions import Cyl
from brainbot.constants import PI, HALF_PI
from brainbot.utils import tf
from brainbot.viz import markers



def main(
        visualize: bool = True
    ):



if __name__ == '__main__':
    Fire(main)
