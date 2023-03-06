import sys
import time
from pathlib import Path
from collections import namedtuple


# sys.path.insert(0, "/home/zak/src/ompl/py-bindings")
# sys.path.insert(0, "/home/zak/src/ompl/build/lib")

from fire import Fire

from ompl import base as ob
import grapeshot as gs

import pybullet as pb

LinkState = namedtuple(
    "LinkState",
    [
        "linkWorldPosition",
        "linkWorldOrientation",
        "localInertialFramePosition",
        "localInertialFrameOrientation",
        "worldLinkFramePosition",
        "worldLinkFrameOrientation",
    ],
)


def main(
        visualize: bool = False
    ):
    # Fetch
    # robot_name = "fetch"
    # group_name = "arm"

    # UR5
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

    # Get ee_link offset for debugging
    # print(f"Num robot links {world.sim.get(0)}")

    print("Links:")
    for l_name, l_id in robot.skel.links.items():
        print(f"{l_name}, id: {l_id}")
        link_state = LinkState(*world.sim.getLinkState(robot.skel.id, l_id, computeForwardKinematics=True))
        world_pos = link_state[4]
        world_orien = link_state[5]
        print(f"    pos: {world_pos}")
        print(f"    orien: {world_orien}")


if __name__ == '__main__':
    Fire(main)
