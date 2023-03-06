import random
import numpy as np
import math
from dataclasses import dataclass

from brainbot.utils import Transform


@dataclass
class Cell:
    lower: np.array  # Vector for lower cell limit
    upper: np.array  # Vector for upper cell limit
    num_attempts = 1
    num_fails = 0

    def setAttempts(self, n):
        self.num_attempts = n

    def setFails(self, n):
        self.num_fails = n


@dataclass
class Cyl:
    radius: float
    height: float
    world_to_center_tf: Transform

    def isInsideTransform(self, world_tf: Transform) -> bool:
        center_to_tf = self.world_to_center_tf.inv() * world_tf
        return self.isInside(center_to_tf.pos)

    def isInside(self, pos: np.array) -> bool:

        # Get radius of tf and return if position is inside vol
        radius_squared = pos[0] ** 2 + pos[1] ** 2
        return radius_squared <= self.radius ** 2 and pos[2] <= self.height/2 and pos[2] >= -self.height/2

    def samplePosInRange(self, lower: np.array, upper: np.array) -> np.array:

        r_squared = random.uniform(
            self.radius ** 2 * lower[0] ** 2, self.radius ** 2 * upper[0] ** 2)
        theta = random.uniform(2 * math.pi * lower[1], 2 * math.pi * upper[1])

        x = math.sqrt(r_squared) * math.cos(theta)
        y = math.sqrt(r_squared) * math.sin(theta)
        z = random.uniform(lower[2] * self.height - self.height /
                           2.0, upper[2] * self.height - self.height/2.0)

        return (x, y, z)

    def sample(self) -> np.array:
        return self.samplePosInRange((0, 0, 0), (1, 1, 1))

    def sampleCartInRange(self, lower, upper) -> np.array:

        while True:
            x = -self.radius + 2 * self.radius * \
                random.uniform(lower[0], upper[0])
            y = -self.radius + 2 * self.radius * \
                random.uniform(lower[1], upper[1])
            z = -self.height/2 + self.height * \
                random.uniform(lower[2], upper[2])

            if self.isInside(x, y, z):
                return (x, y, z)

    def sampleRadialGrid(self, n_r, n_theta, n_h):
        delta_r = self.radius ** 2 / n_r
        delta_theta = 2 * math.pi / n_theta
        delta_h = self.height / n_h

        # Random integer in [0,n]
        r_squared = delta_r * random.randint(1, n_r) ** 2
        # Random integer in [0,n) because 2pi = 0
        theta = delta_theta * random.randrange(0, n_theta)

        x = math.sqrt(r_squared) * math.cos(theta)
        y = math.sqrt(r_squared) * math.sin(theta)

        # Random integer in [0,n]
        z = -self.height/2 + delta_h * random.randint(0, n_h)

        return (x, y, z)

    def sampleCartesianGrid(self, n_x, n_y, n_z):
        delta_x = 2 * self.radius / n_x
        delta_y = 2 * self.radius / n_y
        delta_z = self.height / n_z

        while True:
            x = -self.radius + delta_x * random.randint(0, n_x)
            y = -self.radius + delta_y * random.randint(0, n_y)
            z = -self.height/2 + delta_z * random.randint(0, n_z)

            if self.isInside(x, y, z):
                return (x, y, z)

    def sampleRadialCells(self, bias):
        if not self.cells:
            print("CELLS NOT CONSTRUCTED")
            return (0, 0, 0)

        if random.random() > bias:
            return self.sample()

        total_attempts = sum([c.num_attempts for c in self.cells])

        probs = [c.num_attempts / total_attempts for c in self.cells]
        cur_cell = np.random.choice(self.cells, p=probs)

        return self.sampleRadialCell(cur_cell)

    def sampleCartCells(self, bias):
        if not self.cells:
            print("CELLS NOT CONSTRUCTED")
            return (0, 0, 0)

        if random.random() > bias:
            return self.sample()

        total_attempts = sum([c.num_attempts for c in self.cells])

        # Sampling from a discrete distribution
        probs = [c.num_attempts / total_attempts for c in self.cells]
        cur_cell = np.random.choice(self.cells, p=probs)

        return self.sampleCartCell(cur_cell)

    def sampleRadialCell(self, cell):
        return self.samplePosInRange(cell.lower, cell.upper)

    def sampleCartCell(self, cell):
        return self.sampleCartInRange(cell.lower, cell.upper)

    def constructCells(self, n_r, n_theta, n_h):
        delta_r = 1.0 / n_r
        delta_theta = 1.0 / n_theta
        delta_h = 1.0 / n_h

        self.cells = []

        for r in range(n_r):
            for t in range(n_theta):
                for h in range(n_h):
                    lower = [r * delta_r, t*delta_theta, h*delta_h]
                    upper = [lower[0] + delta_r, lower[1] +
                             delta_theta, lower[2] + delta_h]

                    self.cells.append(Cell(lower, upper))
