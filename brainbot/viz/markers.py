import pybullet as pb
from pybullet_utils.bullet_client import BulletClient

from brainbot.utils import tf
from brainbot.utils.tf import Transform

from collections import namedtuple

TriadIDs = namedtuple("TriadIDs", ["x_id", "y_id", "z_id"])


def displayTriad(sim: BulletClient, trans: Transform, prev_ids = None, length = 0.2, thickness = 2) -> int:

    # Get triad points
    x_pose = trans * tf.toTransform([length, 0, 0], [0,0,0])
    y_pose = trans * tf.toTransform([0, length, 0], [0,0,0])
    z_pose = trans * tf.toTransform([0, 0, length], [0,0,0])

    # If we haven't been passed a prev_ids add points for first time
    if not prev_ids:

        x_line = sim.addUserDebugLine(trans.pos, x_pose.pos, [1,0,0], thickness)
        y_line = sim.addUserDebugLine(trans.pos, y_pose.pos, [0,1,0], thickness)
        z_line = sim.addUserDebugLine(trans.pos, z_pose.pos, [0,0,1], thickness)

        return TriadIDs(x_line, y_line, z_line)

    # Otherwise, replace items by ID
    else:

        sim.addUserDebugLine(trans.pos, x_pose.pos, [1,0,0], thickness, replaceItemUniqueId=prev_ids.x_id)
        sim.addUserDebugLine(trans.pos, y_pose.pos, [0,1,0], thickness, replaceItemUniqueId=prev_ids.y_id)
        sim.addUserDebugLine(trans.pos, z_pose.pos, [0,0,1], thickness, replaceItemUniqueId=prev_ids.z_id)

        return prev_ids
