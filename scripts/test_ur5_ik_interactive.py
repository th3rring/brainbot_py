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
    world.set_world_positions(context.indices,[-3.1,-1.6,1.6,-1.6,-1.6,0.])


    # Get offset transform from tool_tip frame to ee_link grame
    ee_link_state = pb.getLinkState(robot.skel.id, robot.skel.links["ee_link"], computeForwardKinematics=True)
    world_to_ee_t = tf.toTransform(ee_link_state[4],ee_link_state[5])

    tool_tip_state = pb.getLinkState(robot.skel.id, robot.skel.links["tool_tip"], computeForwardKinematics=True)
    world_to_tool_tip_t = tf.toTransform(tool_tip_state[4], tool_tip_state[5])
    tool_tip_triad = markers.displayTriad(world.sim, world_to_tool_tip_t)

    tool_tip_to_ee_t = world_to_tool_tip_t.inv() * world_to_ee_t
    

    # Get robot base transform
    base_state = world.sim.getLinkState(robot.skel.id, robot.skel.links["base_link"], computeForwardKinematics=True)
    world_to_base_t = tf.toTransform(base_state[4], base_state[5])
    base_to_world_t = world_to_base_t.inv()
    print(base_to_world_t)

    # Add sliders for EE pose goal
    x_slider = pb.addUserDebugParameter("x pos",-1,1,0)
    y_slider = pb.addUserDebugParameter("y pos",-1,1,0)
    z_slider = pb.addUserDebugParameter("z pos",0,3,1.1)

    # Add sliders for euler angles
    roll_slider = pb.addUserDebugParameter("x rot",-PI,PI,0)
    pitch_slider = pb.addUserDebugParameter("y rot",-PI,PI,0)
    yaw_slider = pb.addUserDebugParameter("z rot",-PI,PI,0)

    # Create triad for slider position
    slider_triad = markers.displayTriad(world.sim, tf.toTransform([0,0,1.1],[0,0,0]))

    # Add button
    button = pb.addUserDebugParameter("Solve IK", 1,0,1)
    prev_button_val = pb.readUserDebugParameter(button)

    # Transform to and from fk frakes
    fk_offset_t = tf.toTransform([0,0,0], [0, -HALF_PI, HALF_PI] )

    # get ee link
    ee_link_state = world.sim.getLinkState(robot.skel.id, robot.skel.links["ee_link"], computeForwardKinematics=True)
    world_to_ee_t = tf.toTransform(ee_link_state[4],ee_link_state[5])
    ee_triad = markers.displayTriad(world.sim, world_to_ee_t)

    while True:

        # Read values
        x_pos = pb.readUserDebugParameter(x_slider)
        y_pos = pb.readUserDebugParameter(y_slider)
        z_pos = pb.readUserDebugParameter(z_slider)

        x_rot = pb.readUserDebugParameter(roll_slider)
        y_rot = pb.readUserDebugParameter(pitch_slider)
        z_rot = pb.readUserDebugParameter(yaw_slider)

        # Convert to transform
        slider_t = tf.toTransform([x_pos, y_pos, z_pos], [x_rot, y_rot, z_rot])

        # Display triad
        slider_triad = markers.displayTriad(world.sim, slider_t,  slider_triad)

        if (pb.readUserDebugParameter(button) > prev_button_val):

            # Get ik transform for query, this is the body to ee link transform with a rotation offset for the ee
            ik_t = world_to_base_t.inv() * slider_t * tool_tip_to_ee_t * fk_offset_t.inv()  

            inverse_input = ik_t.rigidTF().reshape(-1).tolist()[0]
            print(f"inverse input {inverse_input}")

            # Compute IK
            start = time.time()
            joint_configs = ur5_kin.inverse(inverse_input)
            end = time.time()
            
            n_solutions = int(len(joint_configs)/n_joints)
            print(f"{n_solutions} solutions found in {end-start}s")
            joint_configs = np.asarray(joint_configs).reshape(n_solutions, n_joints)

            for joint_config in joint_configs:

                # Print and set joint config
                print(joint_config)
                world.set_world_positions(context.indices,joint_config)
                
                #Update ee triad
                ee_link_state = world.sim.getLinkState(robot.skel.id, robot.skel.links["ee_link"], computeForwardKinematics=True)
                world_to_ee_t = tf.toTransform(ee_link_state[4],ee_link_state[5])
                ee_triad = markers.displayTriad(world.sim, world_to_ee_t, ee_triad)


                # Update tool tip triad
                tool_tip_state = pb.getLinkState(robot.skel.id, robot.skel.links["tool_tip"], computeForwardKinematics=True)
                world_to_tool_tip_t = tf.toTransform(tool_tip_state[4], tool_tip_state[5])
                tool_tip_triad = markers.displayTriad(world.sim, world_to_tool_tip_t, tool_tip_triad)

                input("Press enter to show next...")

            print("Done.")

            prev_button_val = pb.readUserDebugParameter(button)



if __name__ == '__main__':
    Fire(main)
