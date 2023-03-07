import pybullet as p
import pybullet_data as pd
import os

p.connect(p.DIRECT)
name_in = "cyl_shell_0.25_0.5.obj"
name_out = "cyl_shell_0.25_0.5_vhacd.obj"
name_log = "log.txt"

#  Volumetric Hierarchical Approximate Decomposition
# Converts concave geometry to convex
p.vhacd(name_in, name_out, name_log)
