import sys
import time
from pathlib import Path
import pickle

from fire import Fire
from numpy.random import default_rng
import numpy as np

from ompl import base as ob
import grapeshot as gs
import pybullet as pb

from brainbot.plan.regions import Cyl
from brainbot.utils import tf


class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, cyl: Cyl, si: ob.SpaceInformation, pool: gs.model.WorldPool, robot, group_name, state_to_array):
        super(ValidityChecker, self).__init__(si)
        self.group_name = group_name
        self.pool = pool
        self.robot = robot
        self.cyl = cyl
        self.state_to_array = state_to_array

    def isValid(self, state):

        # Extract underlying state from OMPL state
        state_arr = self.state_to_array(state)

        with self.pool.reserve() as world:
            # Set robot to that state
            world.set_group_positions(
                self.robot.groups[self.group_name], state_arr)

            # Compute forward kinematics
            # Sampling per thread requires this to be their local simulator 
            tool_tip_state = world.sim.getLinkState(
                self.robot.skel.id, self.robot.skel.get_link("tool_tip").index, computeForwardKinematics=True)

            # Get tool tip pose in world frame
            world_to_tool_tip_t = tf.toTransform(
                tool_tip_state[4], tool_tip_state[5])
            
            # Check collision
            ret_val=  not world.in_collision() and self.cyl.isInsideTransform(world_to_tool_tip_t)
            
            # print(f"state valid? {ret_val}")
            
            return ret_val



class RegionSampler(ob.RealVectorStateSampler):
    def __init__(self, data_path, state_to_array, array_to_state, si: ob.SpaceInformation):
        super(RegionSampler, self).__init__(si)
        self.name_ = "region_sampler"

        # Get state getters and setters
        self.state_to_array = state_to_array
        self.array_to_state = array_to_state

        # Load sampler data
        self.rng = default_rng()
        self.data = []
        with open(data_path, 'rb') as f:
            self.data = pickle.load(f)

    def sample(self, state):
        # With some probability, sample from training
        if self.rng.random() < 0.9:
            state_arr = self.state_to_array(state)

            # This can simply be a random int index in the data
            # Find a random point in the data
            i = self.rng.integers(0, len(self.data))
            choice = self.data[i]

            # Old implementation
            # choice = self.rng.choice(
            # np.array(self.data), (1, 1), shuffle=False)[0][0]

            # Add some small noise to the choice
            rand_new = self.rng.normal(choice, scale=0.1, size=(1, 6))

            self.array_to_state(rand_new[0], state)
        else:
            super(RegionSampler, self).sampleUniform(state)

    def sampleUniform(self, state):
        return self.sample(state)


def allocateRegionSampler(state_to_array, array_to_state, si: ob.SpaceInformation):
    return RegionSampler("data/region_0.25_0.5_offangle.pkl", state_to_array, array_to_state, si)


def main(
    planner_name: str = "PRM",
    planning_time: float = 100,
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

    context = gs.planning.get_OMPL_context(
        world, [robot.groups[group_name]], planner_name, 6, sampleAllocatorFunction=allocateRegionSampler)

    # Create cylinder
    cyl = Cyl(0.25, 0.5, tf.toTransform([0, 0, 1.1], [0, 0, 0]))

    # Set validity checker
    svc = ValidityChecker(cyl, context.si, context.pool, robot, group_name, context.state_to_array)
    context.setup.setStateValidityChecker(svc)
    context.set_validity_checking_resolution(0.001)

    # Set sampler
    # context.si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(allocateRegionSampler))

    while True:
        start = ob.State(context.ss)
        goal = ob.State(context.ss)
        
        print("hi")

        while not context.vss.sample(start.get()):
            print("shucks")
            pass
        while not context.vss.sample(goal.get()):
            print("dicks")
            pass
        
        if not svc.isValid(start) or not svc.isValid(goal):
            print("Failed to find valid start and goal states. Retrying.")
            continue

        print("about to plan")
        context.setup.setStartAndGoalStates(start, goal)
        solved = context.setup.solve(planning_time)

        if path := context.get_solution_path(interpolate=100):
            print("Solved!")
            input("Press enter to view solution")
            for state in path.getStates():
                context.set_state(state, world)
                time.sleep(0.1)
            input("Press enter to attempt replanning")
        else:
            print("Failed to find exact solution. Displaying start and goal states.")
            context.set_state(start.get(), world)
            input("Press enter to display goal.")
            context.set_state(goal.get(), world)
            input("Press enter to continue.")

        context.clear_query()


if __name__ == '__main__':
    Fire(main)
