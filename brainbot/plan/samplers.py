import math
from .regions import Cyl
from brainbot.utils import tf
from brainbot.kinematics import Ur5Kinematics
from brainbot.viz import markers
from brainbot.constants import PI

import numpy as np
from numpy.random import default_rng
from typing import Callable
from collections import deque


class DeterministicGridSampler:

    def __init__(self, lower: np.array, upper: np.array, res: np.array):
        # Convert types to numpy
        if not type(lower) == np.array:
            lower = np.array(lower)

        if not type(upper) == np.array:
            upper = np.array(upper)

        # Assert that we are dealing with points in R^3
        assert len(lower) == 3
        assert len(upper) == 3
        assert len(res) == 3

        # Start and stopping point of grid
        self.lower = lower
        self.upper = upper

        # Resolution is # of grid indexes per axis
        self.res = res

        # Calculate the step size per axis
        self.steps = (upper - lower)/res

        # Set current grid point
        self.cur_pos = [0, 0, 0]

    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):

        # Increment x val
        self.cur_pos[0] += 1

        # If x val is past grid x resolution, bring back to 0
        # and increment y
        if self.cur_pos[0] == self.res[0]:
            self.cur_pos[0] = 0
            self.cur_pos[1] += 1

            # Increment y val
            self.cur_pos[1] += 1

            # If y val is past grid y resolution, bring back to 0
            # and increment z
            if self.cur_pos[1] == self.res[1]:
                self.cur_pos[1] = 0
                self.cur_pos[2] += 1

                # Increment z val
                self.cur_pos[2] += 1

                # If we reach z grid resolution, start over
                if self.cur_pos[2] == self.res[2]:
                    self.cur_pos = [0, 0, 0]

        # Return R^3 point
        return self.lower + self.steps * self.cur_pos


class RegionGridSampler:

    def __init__(self, cyl: Cyl, res: np.array):

        self.cyl = cyl
        self.lower_pt = np.array([-cyl.radius, -cyl.radius, -cyl.height/2])
        self.upper_pt = np.array([cyl.radius, cyl.radius, cyl.height/2])

        self.grid_sampler_ = DeterministicGridSampler(
            self.lower_pt, self.upper_pt, res)

    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):

        # Init none var
        next = None
        found_soln = False
        while (not found_soln):

            # Get next point in deterministic sequence
            next = self.grid_sampler_.next()

            # Break out if this point is inside region,
            # otherwise continue in sequence
            if self.cyl.isInside(next):
                found_soln = True
        return next

class RegionSurfaceSampler:
    """
    This class is used to sample points on the surface of a region.
     
    The region is assumed to be a cylinder, and the points are sampled  """

    def __init__(self, cyl: Cyl, res: np.array):
        """Constructor for RegionSurfaceSampler

        Args:
            cyl (Cyl): The cylinder to sample points on
            res (np.array): The resolution of the grid to sample points on. Assumes first element is the circumfrence resolution, and the second element is the height circumfrence resolution. Third element is square grid resolution for the top and bottom of the cylinder.
        """

        self.cyl = cyl
        self.res = res
        self.queue = deque()


    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):

        # If we have queued points, return them
        if len(self.queue) != 0:
            return self.queue.popleft()
        else:
            delta_angle = 2 * PI / self.res[0]
            delta_height = self.cyl.height * 2 / (self.res[1] - 1)

            for h in range(0, self.res[1]):
                # Inside the middle of the cylinder, sample points on the circumference
                if h != 0 and h != self.res[1] - 1:
                    z = h * delta_height - self.cyl.height
                    for i in range(self.res[0]):
                        angle = i * delta_angle

                        # TODO Doesn't really matter, but this could be more efficient
                        x = self.cyl.radius * math.cos(angle)
                        y = self.cyl.radius * math.sin(angle)
                        self.queue.append([x, y, z])
                        
                # Otherwise, we are at the top or bottom of the cylinder
                else:
                    for i in range(self.res[2]):
                        for j in range(self.res[2]):
                            x = i * (2 * self.cyl.radius) / (self.res[2] - 1) - self.cyl.radius
                            y = j * (2 * self.cyl.radius) / (self.res[2] - 1) - self.cyl.radius
                            z = h * delta_height - self.cyl.height
                            
                            if x**2 + y**2 <= self.cyl.radius:
                                self.queue.append([x, y, z])
                    
            # Return next element
            return self.queue.popleft()

class RegionGridConfigSampler:

    def __init__(self, cyl: Cyl, res: np.array, kinematic_solver: Ur5Kinematics, region_validity: Callable):

        self.grid_sampler_ = RegionGridSampler(cyl, res)
        self.validity_ = region_validity
        self.rng_ = default_rng()
        self.kinematic_solver_ = kinematic_solver
        self.queue = deque()

    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):

        # If we have queued configuration solutions, return them
        if len(self.queue) != 0:
            return self.queue.popleft()

        # Otherwise, queue is empty, get next workspace point and
        # queue all joint solutions
        else:
            found_soln = False
            while (not found_soln):

                # Sample grid poing inside region
                pt = self.grid_sampler_.next()

                # Sample valid orientation
                orien = tf.randomQuaternion(top_half_sphere=True)

                # Construct transform to target
                target = self.grid_sampler_.cyl.world_to_center_tf * \
                    tf.toTransform(pt, orien)

                # We care about the EE tool colliding with the region
                # so we call this just to throw out orientations that aren't useful
                # First, call IK on the target
                joint_configs = self.kinematic_solver_.inverse(target)

                if joint_configs is not None and len(joint_configs) != 0:

                    # Call the region_validity function on the first joint config
                    valid = self.validity_(joint_configs[0])

                    # IK solution doesn't put tool in contact with region collision
                    # objects, add joint solutions to queue
                    if valid:

                        # Add results to queue if they exist,
                        for config in joint_configs:
                            self.queue.append(config)

                        # Break out of loop
                        found_soln = True

                # otherwise repeat
            return self.queue.popleft()

class RegionSurfaceConfigSampler:

    def __init__(self, cyl: Cyl, res: np.array, kinematic_solver: Ur5Kinematics):

        self.surface_sampler_ = RegionSurfaceSampler(cyl, res)
        
        # Not checking state validity for surface samples
        # They'll collide with the region collision objects
        # self.validity_ = region_validity

        self.rng_ = default_rng()
        self.kinematic_solver_ = kinematic_solver
        self.queue = deque()

    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):

        # If we have queued configuration solutions, return them
        if len(self.queue) != 0:
            return self.queue.popleft()

        # Otherwise, queue is empty, get next workspace point and
        # queue all joint solutions
        else:
            found_soln = False
            while (not found_soln):

                # Sample grid poing inside region
                pt = self.surface_sampler_.next()

                # Sample valid orientation
                orien = tf.randomQuaternion(top_half_sphere=True)

                # Construct transform to target
                target = self.surface_sampler_.cyl.world_to_center_tf * \
                    tf.toTransform(pt, orien)

                # We care about the EE tool colliding with the region
                # so we call this just to throw out orientations that aren't useful
                # First, call IK on the target
                joint_configs = self.kinematic_solver_.inverse(target)

                if joint_configs is not None and len(joint_configs) != 0:

                    # Removed, see comment in constructor
                    # Call the region_validity function on the first joint config
                    # valid = self.validity_(joint_configs[0])

                    # IK solution doesn't put tool in contact with region collision
                    # objects, add joint solutions to queue
                    # if valid:

                    # Add results to queue if they exist,
                    for config in joint_configs:
                        self.queue.append(config)

                    # Break out of loop
                    found_soln = True

                # otherwise repeat
            return self.queue.popleft()
