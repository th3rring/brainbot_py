import sys
import time
from pathlib import Path

from fire import Fire

from ompl import base as ob
import grapeshot as gs


def main(
        planner_name: str = "RRTConnect",
        planning_time: float = 1.,
        visualize: bool = False
    ):
    robot_name = "fetch"
    group_name = "arm"

    script_path = Path(__file__).parents[0]
    asset_path = script_path / ".." / "assets"
    robot_path = asset_path / robot_name

    world = gs.model.World(visualize)
    robot = world.add_robot(robot_path / f"{robot_name}.urdf", robot_path / f"{robot_name}.srdf")
    env = world.add_environment_yaml(asset_path / "environments" / "shelf_simple.yml")
    world.configure_acm()

    context = gs.ompl.get_OMPL_context(world, [(robot, group_name)], planner_name)

    while True:
        start = ob.State(context.ss)
        goal = ob.State(context.ss)

        while not context.vss.sample(start.get()):
            pass
        while not context.vss.sample(goal.get()):
            pass

        context.setup.setStartAndGoalStates(start, goal)
        solved = context.setup.solve(planning_time)

        if path := context.get_solution_path():
            print("Solved!")
            for state in path.getStates():
                context.set_state(state)
                time.sleep(0.01)
        else:
            print("Failed to find exact solution. Displaying start and goal states.")
            context.set_state(start.get())
            input("Press enter to display goal.")
            context.set_state(goal.get())
            input("Press enter to continue.")

        context.clear_query()


if __name__ == '__main__':
    Fire(main)
