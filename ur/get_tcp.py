from AGRobot import URRobot
import rtde_io
import numpy as np
import open3d as o3d

IP = "10.3.15.95"
rtde_io = rtde_io.RTDEIOInterface(IP)
rob = URRobot(IP)
# debug_l = rob.getjl()

rob.set_tcp([0., 0., 0.07, 0., 0., 0.])
# print(rob.getj())
print(rob.getl())

