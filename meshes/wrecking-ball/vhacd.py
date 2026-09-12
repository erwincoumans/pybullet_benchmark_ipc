import pybullet as p
import pybullet_data as pd
import os
import time

p.connect(p.DIRECT)

name_in = "ball_original.obj"
name_out = "ball.obj"
name_log = "log.txt"

#for more parameters, see http://pybullet.org/guide

p.vhacd(name_in, name_out, name_log, concavity=0, maxNumVerticesPerCH=1024, depth=32, resolution=1000000,convexhullApproximation=0)
