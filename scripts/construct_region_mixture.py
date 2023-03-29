import sys
import time
from pathlib import Path

# sys.path.insert(0, "/home/zak/src/ompl/py-bindings")
# sys.path.insert(0, "/home/zak/src/ompl/build/lib")

from fire import Fire

import numpy as np
from numpy.random import default_rng
from sklearn.mixture import GaussianMixture

from ompl import base as ob

import grapeshot as gs
import pybullet as pb

import ur5_ikfast

from brainbot.kinematics import solvers
from brainbot.plan.regions import Cyl
from brainbot.constants import PI, HALF_PI
from brainbot.plan.samplers import RegionGridConfigSampler
from brainbot.utils import tf
from brainbot.viz import markers


def main(
    visualize: bool = True
):

    robot_name = "ur5_tool"
    group_name = "manipulator"

    script_path = Path(__file__).parents[0]
    asset_path = script_path / ".." / "assets"
    robot_path = asset_path / robot_name

    world = gs.model.World(visualize)
    robot = world.add_robot(
        robot_path / f"{robot_name}.urdf", robot_path / f"{robot_name}.srdf")
    gs.model.process_environment_yaml(
        world.sim, asset_path / "environments" / "cyl_shell_simple.yml")

    world.setup_collision_filter()


    # Create planning context to get joint indicies
    planner_name = "RRTConnect"
    context = gs.planning.get_OMPL_context(world, [robot.groups[group_name]], planner_name)


    # Set initial configuration
    initial_config = [-3.1, 1.6, 1.6, -1.6, -1.6, 0.]
    world.set_group_positions(robot.groups[group_name],initial_config)


    # Get offset transform from tool_tip frame to ee_link grame
    ee_link_state = world.sim.getLinkState(
        robot.skel.id, robot.skel.get_link["ee_link"].index, computeForwardKinematics=True)
    world_to_ee_t = tf.toTransform(ee_link_state[4], ee_link_state[5])

    # Get robot base transform
    base_state = world.sim.getLinkState(
        robot.skel.id, robot.skel.get_link["base_link"].index, computeForwardKinematics=True)
    world_to_base_t = tf.toTransform(base_state[4], base_state[5])

    tool_tip_state = pb.getLinkState(
        robot.skel.id, robot.skel.get_link["tool_tip"].index, computeForwardKinematics=True)
    world_to_tool_tip_t = tf.toTransform(tool_tip_state[4], tool_tip_state[5])
    tool_tip_triad = markers.displayTriad(world.sim, world_to_tool_tip_t)

    tool_tip_to_ee_t = world_to_tool_tip_t.inv() * world_to_ee_t

    # Create Ur5 Kinematics obj
    ur5_kin = solvers.Ur5Kinematics(tool_tip_to_ee_t.inv(), world_to_base_t)

    # Create cylinder
    cyl = Cyl(0.25, 0.5, tf.toTransform([0, 0, 1.1], [0, 0, 0]))

    # Create validity checker function
    def isValid(state):
        world.set_group_positions(robot.groups[group_name],state)
        return not world.in_collision()

    # Create sampler
    s = RegionGridConfigSampler(cyl, [30, 30, 10], ur5_kin, isValid)

    num_data = 90000
    num_components = 300

    X = []
    mixture = []
    for _ in range(num_data):
        X.append(s.next())

    rng = default_rng()

    def drawSample():
        choice = rng.choice(np.array(X), (1, 1), shuffle=False)[0][0]
        rand_new = rng.normal(choice, scale=0.1, size=(1, 6))
        return rand_new

    # gm = GaussianMixture(n_components=num_components,
    #                      init_params='random_from_data', covariance_type='tied').fit(np.array(X))
    print("Fix GMM!")

    while (True):
        # config, label = gm.sample()
        config = drawSample()
        config = config[0]
        print(f"Sampled {config}")
        world.set_world_positions(context.indices, config)

        input("Press enter to display next config...")


if __name__ == '__main__':
    Fire(main)
