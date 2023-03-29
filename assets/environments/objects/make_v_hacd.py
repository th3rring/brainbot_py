import pybullet as p
import pybullet_data as pd
import os

p.connect(p.DIRECT)
name_in =  "cyl_shell_0.25_0.5.obj"
name_out = "cyl_shell_0.25_0.5_vhacd.obj"
name_log = "log.txt"

#  Volumetric Hierarchical Approximate Decomposition 
# Converts concave geometry to convex, https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.eqlo4t4ozi7
p.vhacd(name_in, name_out, name_log)

