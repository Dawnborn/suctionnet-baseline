from turtle import home
from AGRobot import URRobot
import rtde_io
import time
import numpy as np
import math
import open3d as o3d

IP = "10.3.15.95"
rtde_io = rtde_io.RTDEIOInterface(IP)
rob = URRobot(IP)
# debug_l = rob.getjl()

rob.set_tcp([0., 0., 0.185, 0., 0., 0.])

# sucker on
def sucker_on():
    rtde_io.setStandardDigitalOut(0, True)
    rtde_io.setStandardDigitalOut(1, False)
    time.sleep(1.0)

# sucker off
def sucker_off():
    rtde_io.setStandardDigitalOut(1, True)
    rtde_io.setStandardDigitalOut(0, False)
    time.sleep(0.5)

_MAX_FLOAT = np.maximum_sctype(np.float64)
_FLOAT_EPS = np.finfo(np.float64).eps

	

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()



def axangle2mat(axis, angle, is_normalized=False):
    ''' Rotation matrix for rotation angle `angle` around `axis`

    Parameters
    ----------
    axis : 3 element sequence
       vector specifying axis for rotation.
    angle : scalar
       angle of rotation in radians.
    is_normalized : bool, optional
       True if `axis` is already normalized (has norm of 1).  Default False.

    Returns
    -------
    mat : array shape (3,3)
       rotation matrix for specified rotation

    Notes
    -----
    From: http://en.wikipedia.org/wiki/Rotation_matrix#Axis_and_angle
    '''
    x, y, z = axis
    if not is_normalized:
        n = math.sqrt(x * x + y * y + z * z)
        x = x / n
        y = y / n
        z = z / n
    c = math.cos(angle);
    s = math.sin(angle);
    C = 1 - c
    xs = x * s;
    ys = y * s;
    zs = z * s
    xC = x * C;
    yC = y * C;
    zC = z * C
    xyC = x * yC;
    yzC = y * zC;
    zxC = z * xC
    return np.array([
        [x * xC + c, xyC - zs, zxC + ys],
        [xyC + zs, y * yC + c, yzC - xs],
        [zxC - ys, yzC + xs, z * zC + c]])


def mat2axangle(mat, unit_thresh=1e-5):
    """Return axis, angle and point from (3, 3) matrix `mat`

    Parameters
    ----------
    mat : array-like shape (3, 3)
        Rotation matrix
    unit_thresh : float, optional
        Tolerable difference from 1 when testing for unit eigenvalues to
        confirm `mat` is a rotation matrix.

    Returns
    -------
    axis : array shape (3,)
       vector giving axis of rotation
    angle : scalar
       angle of rotation in radians.

    Examples
    --------
    >>> direc = np.random.random(3) - 0.5
    >>> angle = (np.random.random() - 0.5) * (2*math.pi)
    >>> R0 = axangle2mat(direc, angle)
    >>> direc, angle = mat2axangle(R0)
    >>> R1 = axangle2mat(direc, angle)
    >>> np.allclose(R0, R1)
    True

    Notes
    -----
    http://en.wikipedia.org/wiki/Rotation_matrix#Axis_of_a_rotation
    """
    M = np.asarray(mat, dtype=np.float)
    # direction: unit eigenvector of R33 corresponding to eigenvalue of 1
    L, W = np.linalg.eig(M.T)
    print(L)
    i = np.where(np.abs(L - 1.0) < unit_thresh)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    direction = np.real(W[:, i[-1]]).squeeze()
    # rotation angle depending on direction
    cosa = (np.trace(M) - 1.0) / 2.0
    if abs(direction[2]) > 1e-8:
        sina = (M[1, 0] + (cosa - 1.0) * direction[0] * direction[1]) / direction[2]
    elif abs(direction[1]) > 1e-8:
        sina = (M[0, 2] + (cosa - 1.0) * direction[0] * direction[2]) / direction[1]
    else:
        sina = (M[2, 1] + (cosa - 1.0) * direction[1] * direction[2]) / direction[0]
    angle = math.atan2(sina, cosa)
    return direction, angle

home_joint = [0.0785517692565918, -1.3227990430644532, -1.5260076522827148, -1.8056289158263148, 1.6004223823547363, -4.512999359761373]

hand_eye = np.array([[ 0.97930821, -0.0224255,  -0.20112815,  0.49894768],
 [-0.01984581 ,-0.99969301,  0.01483358, -0.35326488],
 [-0.20139906 ,-0.01053509, -0.97945262,  1.13278423],
 [ 0.         , 0.        ,  0.        ,  1.        ],])

# rob.movel([0,0,0.1,0,0,0], relative=True)
# rob.movej([-1.4326751867877405, -1.5287020963481446, -1.097041130065918, -1.988598962823385, 1.5596418380737305, -2.850908104573385])
# pose_cam = np.array([[0.939927,0,-0.34137551, 0.08491191101074219,],
#                     [-0.04139008,0.99262264,-0.11396147, 0.05025838088989258],
#                     [0.33885705,0.12124503, 0.93299282,1.0345941162109376],
#                     [0, 0, 0, 1]])
# print(np.linalg.det(pose_cam[:3,:3]))
# pose_rob = np.dot(hand_eye, pose_cam )


mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.2, )
mesh_frame2 = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=1.0, )
mesh_frame3 = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.6, )
# mesh_frame4_cam = o3d.geometry.TriangleMesh.create_coordinate_frame(
#     size=0.6, )
mesh_frame2.transform(hand_eye)
# mesh_frame4_cam.transform(pose_cam)
pcd = o3d.io.read_point_cloud("/home/lz/Downloads/0.ply")
pcd.scale(0.001, [0,0,0])
pcd.transform(hand_eye)
    
points_list_index = pick_points(pcd)
pcd_np = np.asanyarray(pcd.points)
pcd_normal_np = np.asanyarray(pcd.normals)
for i in range(len(points_list_index)):
    print("position")
    print(pcd_np[points_list_index[i]])
    print("normal")
    print(pcd_normal_np[points_list_index[i]])
mask = []
for i in range(len(pcd_np)):
    if i == points_list_index[0]:
        index_new = mask.count(1)
    dist = np.linalg.norm(pcd_np[points_list_index[0]] - pcd_np[i])
    if dist < 0.01:
        mask.append(1)
    else:
        mask.append(0)
pcd.points = o3d.utility.Vector3dVector(pcd_np[np.asarray(mask).astype(bool)])
pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=100))

# nx, ny, nz = pcd_normal_np[points_list_index[0]]
nx, ny, nz = np.asanyarray(pcd.normals)[index_new]
# nx, ny, nz = sum(np.asanyarray(pcd.normals)) / len(np.asanyarray(pcd.normals))
print("nx, ny, ,nz")
nx = -nx
ny = -ny
nz = -nz
print(nx)
print(ny)
print(nz)
zd = np.array([nx, ny, nz])
zd = zd / np.linalg.norm(zd)
yd = np.array([0,1,0])
xd = np.cross(yd, zd)
xd = xd / np.linalg.norm(xd)
yd = np.cross(zd, xd)
R = np.array([xd, yd, zd])
print("R: ", R)
pose_rob = np.zeros((4,4))
pose_rob[3,3] = 1
pose_rob[:3,:3] = R.T
pose_rob[:3,3] = pcd_np[points_list_index[0]]
print(pose_rob)
print(np.linalg.det(pose_rob[:3,:3]))

mesh_frame3.transform(pose_rob)

o3d.visualization.draw_geometries([pcd, mesh_frame, mesh_frame2, mesh_frame3])

# pcd = o3d.io.read_point_cloud("./0.ply")
# pcd.scale(0.001, [0,0,0])
# o3d.visualization.draw_geometries([pcd, mesh_frame, mesh_frame4_cam])

# print(rob.getl())
print(rob.getj())
rob.movej(home_joint)


axis, angle = mat2axangle(pose_rob[:3,:3])
T = pose_rob[:3,3]
xyzrxryrz = np.concatenate((T, axis[:3] * angle), axis=0)
print(xyzrxryrz)
rob.movel(xyzrxryrz + [0, 0, 0.02, 0, 0 , 0], vel=0.1)
rob.movel(xyzrxryrz + [0, 0, 0.00, 0, 0 , 0], vel=0.1)
sucker_on()
rob.movel(xyzrxryrz + [0, 0, 0.05, 0, 0 , 0], vel=0.1)
rob.movej(home_joint, vel = 0.2)
sucker_off()


#
# gripper = Robotiq(rob)
# gripper.set_force(50)
# gripper.set_speed(100)
# gripper.close()
# print("object detection : {}".format(gripper.getRobotiqObjectDetected()))
# import time
# # time.sleep(3)
# for i in range(2):
#     result = gripper.move(0)
#     print("object detection : {}".format(gripper.getRobotiqObjectDetected()))
#     position = gripper.get_position()
#     print("object detection : {}".format(gripper.getRobotiqObjectDetected()))
#     gripper.move(85)
#     print("object detection : {}".format(gripper.getRobotiqObjectDetected()))

rob.stopScript()