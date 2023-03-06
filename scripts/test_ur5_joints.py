import sys
import time
from pathlib import Path

# sys.path.insert(0, "/home/zak/src/ompl/py-bindings")
# sys.path.insert(0, "/home/zak/src/ompl/build/lib")

from fire import Fire

from ompl import base as ob
import grapeshot as gs

import pybullet as pb


def main(
        visualize: bool = False
    ):
    robot_name = "ur5_tool"
    group_name = "manipulator"

    script_path = Path(__file__).parents[0]
    asset_path = script_path / ".." / "assets"
    robot_path = asset_path / robot_name

    world = gs.model.World(visualize)
    robot = world.add_robot(robot_path / f"{robot_name}.urdf", robot_path / f"{robot_name}.srdf")
    env = world.add_environment_yaml(asset_path / "environments" / "table_simple.yml")
    world.configure_acm()

    print(f"Added robot with ID {robot.skel.id}")

    input("Press Enter to continue...")

    # Get ee_link offset for debugging
    # ee_link_state = world.sim.getLinkState(robot.skel.id, robot.skel.links["ee_link"], computeForwardKinematics=1)
    # print(ee_link_state)
    # ee_link_world_t = tf.toTransform(ee_link_state.)

    print("UR5 Joints:")
    for l_name, l_id in robot.skel.joints.items():
        print(f"{l_name}, id: {l_id}")

        # Have to decode byte data from pybullet
        joint_info = [i.decode() if isinstance(i, bytes) else i for i in world.sim.getJointInfo(robot.skel.id, l_id)]
        print(f"state: {joint_info}")

if __name__ == '__main__':
    Fire(main)
