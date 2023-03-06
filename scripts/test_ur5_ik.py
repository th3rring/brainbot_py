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

from brainbot.constants import PI, HALF_PI
from brainbot.utils import tf
from brainbot.viz import markers


def main(
        visualize: bool = True
    ):
    ur5_kin = ur5_ikfast.PyKinematics("ur5")
    n_joints = ur5_kin.getDOF()

    robot_name = "ur5_tool"
    group_name = "manipulator"

    script_path = Path(__file__).parents[0]
    asset_path = script_path / ".." / "assets"
    robot_path = asset_path / robot_name

    world = gs.model.World(visualize)
    robot = world.add_robot(robot_path / f"{robot_name}.urdf", robot_path / f"{robot_name}.srdf")
    world.configure_acm()

    # Create planning context to get joint indicies
    planner_name  = "RRTConnect"
    context = gs.ompl.get_OMPL_context(world, [(robot, group_name)], planner_name)

    # Set initial configuration
    initial_config=[-3.1,1.6,1.6,-1.6,-1.6,0.]
    world.set_world_positions(context.indices,initial_config)


    # Get offset transform from tool_tip frame to ee_link grame
    ee_link_state = world.sim.getLinkState(robot.skel.id, robot.skel.links["ee_link"], computeForwardKinematics=True)
    world_to_ee_t = tf.toTransform(ee_link_state[4],ee_link_state[5])


    # Get robot base transform
    base_state = world.sim.getLinkState(robot.skel.id, robot.skel.links["base_link"], computeForwardKinematics=True)
    world_to_base_t = tf.toTransform(base_state[4], base_state[5])


    tool_tip_state = pb.getLinkState(robot.skel.id, robot.skel.links["tool_tip"], computeForwardKinematics=True)
    world_to_tool_tip_t = tf.toTransform(tool_tip_state[4], tool_tip_state[5])
    tool_tip_triad = markers.displayTriad(world.sim, world_to_tool_tip_t)

    tool_tip_to_ee_t = world_to_tool_tip_t.inv() * world_to_ee_t


    # fk_val = ur5_kin.forward(initial_config, 'matrix')
    fk_val = ur5_kin.forward(initial_config)
    fk_val  = np.asarray(fk_val).reshape(3,4) # 3x4 rigid transformation matrix

    fk_t = tf.transformMatrixToTransform(fk_val)

    fk_offset_t = tf.toTransform([0,0,0], [0, -HALF_PI, HALF_PI] )

    fk_triad = markers.displayTriad(world.sim, world_to_base_t * fk_t * fk_offset_t * tool_tip_to_ee_t.inv())
    
    print(f"FK EE world pose {world_to_base_t * fk_t * fk_offset_t}")

    print("Showing FK triad")
    input("Press Enter to close...")

    world_to_ee_triad = markers.displayTriad(world.sim, world_to_ee_t)

    print(f"Pybullet FK EE world pose {world_to_ee_t}")

    print("Showing pybullet FK triad")

    ik_t = world_to_base_t.inv() * world_to_ee_t * fk_offset_t.inv()  

    inverse_input = ik_t.rigidTF().reshape(-1).tolist()[0]

    print(f"inverse input {inverse_input}")

    joint_configs = ur5_kin.inverse(inverse_input)

    n_solutions = int(len(joint_configs)/n_joints)
    print("%d solutions found:" % (n_solutions))
    joint_configs = np.asarray(joint_configs).reshape(n_solutions, n_joints)
    for joint_config in joint_configs:
        print(joint_config)
        world.set_world_positions(context.indices,joint_config)
        input("...")





    # Call IK and set each one

    # n_solutions = int(len(joint_configs)/n_joints)
    # print("%d solutions found:"%(n_solutions))
    # if n_solutions != 0:
    #     joint_configs = np.asarray(joint_configs).reshape(n_solutions,6)
    #     print(f"joint configs: {joint_configs}")

    #     # Set robot to first joint solution
    #     world.set_world_positions(context.indices, joint_configs[0])

    input("Press Enter to close...")

if __name__ == '__main__':
    Fire(main)
