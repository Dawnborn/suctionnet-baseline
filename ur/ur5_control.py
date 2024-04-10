#%%
import rtde_control
import rtde_receive
import rtde_io
import time
import numpy as np

from scipy.spatial.transform import Rotation as R

def z_axis_to_quaternion(z_axis):
    # 确保z_axis是单位向量
    z_axis = z_axis / np.linalg.norm(z_axis)
    
    # 选择一个默认上方向，计算x_axis
    y_axis_default = np.array([0, 1, 0])
    x_axis = np.cross(y_axis_default, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)  # 归一化
    
    # 计算y_axis
    y_axis = np.cross(z_axis, x_axis)
    
    # 从轴构建旋转矩阵
    rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
    
    # 将旋转矩阵转换为四元数
    quaternion = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
    
    return quaternion

#%%
# GET CURRENT POSITION
ip_robot = "10.3.15.95"
rtde_r = rtde_receive.RTDEReceiveInterface(ip_robot)
rtde_c = rtde_control.RTDEControlInterface(ip_robot)
rtde_io_ = rtde_io.RTDEIOInterface(ip_robot)

actual_q = rtde_r.getActualQ() # joint position
actual_tcp = rtde_r.getActualTCPPose()  # pos:xyz,quat: 
print(actual_tcp)
print(actual_q)

#%% 
# CONTROLL THE GECKO
# rtde_io_.setStandardDigitalOut(1, True)
rtde_io_.setStandardDigitalOut(0, True) # 吸取
print("Standard digital out (0) is {}".format(rtde_r.getDigitalOutState(0)))
print("Tool digital out (1) is {}".format(rtde_r.getDigitalOutState(1)))
time.sleep(2)

rtde_io_.setStandardDigitalOut(0, False) # 松开
# rtde_io_.setStandardDigitalOut(1, False)

print("Standard digital out (0) is {}".format(rtde_r.getDigitalOutState(0)))
print("Tool digital out (1) is {}".format(rtde_r.getDigitalOutState(1)))

#%% move tcp
    
# init_q = actual_q
# # Target in the robot base
# new_q = init_q[:]
# new_q[0] -= 0.10

# rtde_c.moveJ(new_q, 1.05, 1.4, True)
# time.sleep(0.2)

# Stop the RTDE control script
rtde_c.stopScript()


