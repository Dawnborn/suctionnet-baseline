import os
import torch
import time
import numpy as np
import open3d as o3d
import torch.nn as nn
import scipy.io as scio
import argparse
import DeepLabV3Plus.network as network
import ConvNet
import torch.nn.functional as F
from PIL import Image
import cv2

import pykinect_azure as pykinect
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R
import time
import math
import datetime

import rtde_control
import rtde_receive
import rtde_io

# GET CURRENT POSITION
ip_robot = "10.3.15.95"
rtde_r = rtde_receive.RTDEReceiveInterface(ip_robot)
rtde_c = rtde_control.RTDEControlInterface(ip_robot)
rtde_io_ = rtde_io.RTDEIOInterface(ip_robot)

rtde_c.setTcp([0., 0., 0.177, 0., 0., 0.])

home_joint = [0.0785517692565918, -1.3227990430644532, -1.5260076522827148, -1.8056289158263148, 1.6004223823547363, -4.512999359761373]
# home_joint = [[0.9231476783752441, -1.7742535076537074, -1.5310468673706055, -1.3154333394816895, 1.5701546669006348, -6.127761665974752]]
target_joint = [1.0150766372680664, -1.8237129650511683, -1.5469045639038086, -1.2950390142253418, 1.5544476509094238, -4.155783478413717]

def analyze_transform_matrix(z_axis_vector, translation_z):
    """
    Analyzes z_axis_vector and translation_z.
    """
    # z_axis_vector = transform_matrix[0:3, 2]
    world_z_axis_vector = np.array([0, 0, 1])
    z_axis_vector = z_axis_vector / np.linalg.norm(z_axis_vector)
    dot_product = np.dot(z_axis_vector, world_z_axis_vector)
    angle_cosine = dot_product
    is_acute_angle = angle_cosine > 0.7
    # translation_z = transform_matrix[3, 2]
    is_above_z0_plane = translation_z > 0.04
    return (is_acute_angle, is_above_z0_plane)

def z_axis_to_rotation_matrix(z_axis):
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
    # quaternion = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
    
    return rotation_matrix

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
    M = mat
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

# sucker on
def sucker_on():
    rtde_io_.setStandardDigitalOut(0, True)
    rtde_io_.setStandardDigitalOut(1, False)
    time.sleep(1.0)

# sucker off
def sucker_off():
    rtde_io_.setStandardDigitalOut(1, True)
    rtde_io_.setStandardDigitalOut(0, False)
    time.sleep(0.5)

def move_robot_home():
    actual_q = rtde_r.getActualQ() # joint position
    actual_tcp = rtde_r.getActualTCPPose()
    print(actual_tcp)

    rtde_c.moveJ(home_joint)
    time.sleep(2)

def move_robot(new_tcp, new_normal=np.array([0,0,1])):
    actual_q = rtde_r.getActualQ() # joint position
    actual_tcp = rtde_r.getActualTCPPose()
    print(actual_tcp)

    rtde_c.moveJ(home_joint)
    time.sleep(2)

    offset = np.zeros(6)
    offset[:3]+=0.2*new_normal

    rtde_c.moveL(new_tcp+offset, speed=0.1)
    time.sleep(2)
    rtde_c.moveL(new_tcp, speed=0.1)
    time.sleep(2)

    # rtde_c.moveJ(home_joint)
    sucker_on()
    rtde_c.moveL(new_tcp+offset, speed=0.1)
    time.sleep(2)
    rtde_c.moveJ(home_joint)
    time.sleep(2)
    rtde_c.moveJ(target_joint)
    time.sleep(2)
    sucker_off()

def find_top_k_ids(arr, k):
    """
    找到数组中最大的k个数对应的索引。

    参数:
    arr: 一维numpy数组。
    k: 需要找到的最大数的数量。

    返回:
    top_k_ids: 最大的k个数的索引数组，按照从大到小的顺序排列。
    """
    if k <= 0:
        return np.array([])  # 如果k<=0，返回空数组
    if k > arr.size:
        # 如果k大于数组的大小，返回整个数组索引，先按值排序，再反转以保证从大到小的顺序
        return np.argsort(arr)[::-1]
    
    # 使用argpartition找到最大的k个数的索引
    top_k_ids_unsorted = np.argpartition(arr, -k)[-k:]
    
    # 对找到的k个索引对应的值进行排序，获取正确的顺序
    top_k_ids = top_k_ids_unsorted[np.argsort(arr[top_k_ids_unsorted])][::-1]

    return top_k_ids

def transform_normals(normals,transformation_matrix):
    rot = transformation_matrix[:3,:3]
    transformed_normals = np.dot(normals, rot.T)
    return transformed_normals

def transform_points(points, transformation_matrix):
    """
    使用4x4齐次转换矩阵转换n*3的点集合坐标。

    参数:
    points: numpy数组，形状为(n, 3)，表示点集合。
    transformation_matrix: numpy数组，形状为(4, 4)，表示齐次转换矩阵。

    返回:
    transformed_points: 转换后的点集合，numpy数组，形状为(n, 3)。
    """
    # 将点集合转换为齐次坐标
    homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))
    
    # 应用转换矩阵
    transformed_homogeneous_points = np.dot(homogeneous_points, transformation_matrix.T)
    
    # 将转换后的齐次坐标点转换回普通的3D坐标点
    transformed_points = transformed_homogeneous_points[:, :3]
    
    return transformed_points

def undistort_image(image, K, dist_coeffs):
    h, w = image.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(image, K, dist_coeffs, None, new_K)
    return undistorted_image

def undistort_depth(depth, K, dist_coeffs):
    h, w = depth.shape[:2]
    mapx, mapy = cv2.initUndistortRectifyMap(K, dist_coeffs, None, K, (w, h), cv2.CV_32FC1)
    undistorted_depth = cv2.remap(depth, mapx, mapy, cv2.INTER_LINEAR)
    return undistorted_depth

parser = argparse.ArgumentParser()
parser.add_argument('--model', default='deeplabv3plus_resnet101', help='Model file name [default: votenet]')
parser.add_argument("--num_classes", type=int, default=2)
parser.add_argument("--output_stride", type=int, default=16, choices=[8, 16])
parser.add_argument('--log_dir', default='log_inf', help='Dump dir to save model checkpoint [default: log]')
parser.add_argument('--split', default='test_seen', help='dataset split [default: test_seen]')
parser.add_argument('--camera', default='kinect', help='camera to use [default: kinect]')
parser.add_argument('--dataset_root', default='/DATA2/Benchmark/graspnet', help='where dataset is')
parser.add_argument('--save_dir', 
                    default='/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/inference_output_grasping4', 
                    help='Dump dir to save model checkpoint [default: log]')
parser.add_argument('--checkpoint_path', 
                    default='/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/ckpt/kinect-deeplabplus-RGBD', 
                    help='Model checkpoint path [default: None]')
parser.add_argument('--overwrite', action='store_true', help='Overwrite existing log and dump folders.')
parser.add_argument('--save_visu', default=True, help='whether to save visualizations.')
FLAGS = parser.parse_args()


LOG_DIR = FLAGS.log_dir
CHECKPOINT_PATH = FLAGS.checkpoint_path
SAVE_PATH = FLAGS.save_dir
split = FLAGS.split
camera = FLAGS.camera
scene_list = []

# # Prepare LOG_DIR and DUMP_DIR
# if os.path.exists(LOG_DIR) and FLAGS.overwrite:
#     print('Log folder %s already exists. Are you sure to overwrite? (Y/N)'%(LOG_DIR))
#     c = input()
#     if c == 'n' or c == 'N':
#         print('Exiting..')
#         exit()
#     elif c == 'y' or c == 'Y':
#         print('Overwrite the files in the log and dump folers...')
#         os.system('rm -r %s'%(LOG_DIR))

# if not os.path.exists(LOG_DIR):
#     os.mkdir(LOG_DIR)

# LOG_FOUT = open(os.path.join(LOG_DIR, 'log_test.txt'), 'a')
# LOG_FOUT.write(str(FLAGS)+'\n')

# def log_string(out_str):
#     LOG_FOUT.write(out_str+'\n')
#     LOG_FOUT.flush()
#     print(out_str)

def show_image(image):
    # 读取图像文件
    
    # 检查图像是否成功读取
    if image is None:
        print("Error: Unable to read image.")
        return
    
    # 创建窗口并显示图像
    cv2.imshow('Image', image[:,:,[2,1,0]])
    
    # 等待用户按下任意键
    cv2.waitKey(0)
    
    # 销毁创建的窗口
    cv2.destroyAllWindows()

def my_worker_init_fn(worker_id):
    np.random.seed(np.random.get_state()[1][0] + worker_id)
    pass


model_map = {
        'deeplabv3_resnet50': network.deeplabv3_resnet50,
        'deeplabv3plus_resnet50': network.deeplabv3plus_resnet50,
        'deeplabv3_resnet101': network.deeplabv3_resnet101,
        'deeplabv3plus_resnet101': network.deeplabv3plus_resnet101,
        'deeplabv3_mobilenet': network.deeplabv3_mobilenet,
        'deeplabv3plus_mobilenet': network.deeplabv3plus_mobilenet,
        'convnet_resnet101': ConvNet.convnet_resnet101,
        'deeplabv3plus_resnet101_depth': network.deeplabv3plus_resnet101_depth
    }
net = model_map[FLAGS.model](num_classes=FLAGS.num_classes, output_stride=FLAGS.output_stride)
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
net = nn.DataParallel(net)
net.to(device)
   

EPOCH_CNT = 0
if CHECKPOINT_PATH is not None and os.path.isfile(CHECKPOINT_PATH):
    print('Loading model from:')
    print(CHECKPOINT_PATH)
    checkpoint = torch.load(CHECKPOINT_PATH)
    
    net.load_state_dict(checkpoint['model_state_dict'])

    EPOCH_CNT = checkpoint['epoch']

def uniform_kernel(kernel_size):
    kernel = np.ones((kernel_size, kernel_size), dtype=np.float32)
    # center = kernel_size // 2
    kernel = kernel / kernel_size**2

    return kernel

class CameraInfo():
    def __init__(self, width, height, fx, fy, cx, cy, scale):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.scale = scale

def create_point_cloud_from_depth_image(depth, camera, organized=True):
    assert(depth.shape[0] == camera.height and depth.shape[1] == camera.width)
    xmap = np.arange(camera.width)
    ymap = np.arange(camera.height)
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depth
    points_x = (xmap - camera.cx) * points_z / camera.fx
    points_y = (ymap - camera.cy) * points_z / camera.fy
    cloud = np.stack([points_x, points_y, points_z], axis=-1)
    if not organized:
        cloud = cloud.reshape([-1, 3])
    return cloud

def grid_sample(pred_score_map, down_rate=20, topk=512):
    num_row = pred_score_map.shape[0] // down_rate
    num_col = pred_score_map.shape[1] // down_rate

    idx_list = []
    for i in range(num_row):
        for j in range(num_col):
            pred_score_grid = pred_score_map[i*down_rate:(i+1)*down_rate, j*down_rate:(j+1)*down_rate]
            
            max_idx = np.argmax(pred_score_grid)
            max_idx = np.array([max_idx // down_rate, max_idx % down_rate]).astype(np.int32)
            
            max_idx[0] += i*down_rate
            max_idx[1] += j*down_rate
            idx_list.append(max_idx[np.newaxis, ...])
    
    idx = np.concatenate(idx_list, axis=0)
    suction_scores = pred_score_map[idx[:, 0], idx[:, 1]]
    sort_idx = np.argsort(suction_scores)
    sort_idx = sort_idx[::-1]

    sort_idx_topk = sort_idx[:topk]

    suction_scores_topk = suction_scores[sort_idx_topk]
    idx0_topk = idx[:, 0][sort_idx_topk]
    idx1_topk = idx[:, 1][sort_idx_topk]

    return suction_scores_topk, idx0_topk, idx1_topk

def drawGaussian(img, pt, score, sigma=1):
    """Draw 2d gaussian on input image.
    Parameters
    ----------
    img: torch.Tensor
        A tensor with shape: `(3, H, W)`.
    pt: list or tuple
        A point: (x, y).
    sigma: int
        Sigma of gaussian distribution.
    Returns
    -------
    torch.Tensor
        A tensor with shape: `(3, H, W)`.
    """
    # img = to_numpy(img)
    tmp_img = np.zeros([img.shape[0], img.shape[1]], dtype=np.float32)
    tmpSize = 3 * sigma
    # Check that any part of the gaussian is in-bounds
    ul = [int(pt[0] - tmpSize), int(pt[1] - tmpSize)]
    br = [int(pt[0] + tmpSize + 1), int(pt[1] + tmpSize + 1)]

    if (ul[0] >= img.shape[1] or ul[1] >= img.shape[0] or br[0] < 0 or br[1] < 0):
        # If not, just return the image as is
        return img

    # Generate gaussian
    size = 2 * tmpSize + 1
    x = np.arange(0, size, 1, float)
    y = x[:, np.newaxis]
    x0 = y0 = size // 2
    
    # Usable gaussian range
    g_x = max(0, -ul[0]), min(br[0], img.shape[1]) - ul[0]
    g_y = max(0, -ul[1]), min(br[1], img.shape[0]) - ul[1]
    img_x = max(0, ul[0]), min(br[0], img.shape[1])
    img_y = max(0, ul[1]), min(br[1], img.shape[0])

    g = np.exp(- ((x - x0) ** 2 + (y - y0) ** 2) / (2 * sigma ** 2)) * score
    g = g[g_y[0]:g_y[1], g_x[0]:g_x[1]]

    tmp_img[img_y[0]:img_y[1], img_x[0]:img_x[1]] = g
    img += tmp_img

def get_suction_from_heatmap(depth_img, heatmap, camera_info):
    suction_scores, idx0, idx1 = grid_sample(heatmap, down_rate=10, topk=1024)

    if len(depth_img.shape) == 3:
        depth_img = depth_img[..., 0]
    point_cloud = create_point_cloud_from_depth_image(depth_img, camera_info)
    
    suction_points = point_cloud[idx0, idx1, :]

    tic_sub = time.time()
    
    point_cloud = point_cloud.reshape(-1, 3)
    pc_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(point_cloud))
    pc_voxel_sampled = pc_o3d.voxel_down_sample(0.003)
    points_sampled = np.array(pc_voxel_sampled.points).astype(np.float32)
    points_sampled = np.concatenate([suction_points, points_sampled], axis=0)
    pc_voxel_sampled.points = o3d.utility.Vector3dVector(points_sampled)
    pc_voxel_sampled.estimate_normals(o3d.geometry.KDTreeSearchParamRadius(0.015), fast_normal_computation=False)
    pc_voxel_sampled.orient_normals_to_align_with_direction(np.array([0., 0., -1.]))
    pc_voxel_sampled.normalize_normals()
    pc_normals = np.array(pc_voxel_sampled.normals).astype(np.float32)
    suction_normals = pc_normals[:suction_points.shape[0], :]

    toc_sub = time.time()
    print('estimate normal time:', toc_sub - tic_sub)

    suction_arr = np.concatenate([suction_scores[..., np.newaxis], suction_normals, suction_points], axis=-1)

    return suction_arr, idx0, idx1

def inference_one_view(camera_info, rgb, depth, scene_idx, anno_idx, hand_eye_m=np.eye(4), env_mask=1, top_k=10):
    """
    return: suction_points, suction_normals, suction_scores
    """
    # rgb = cv2.imread(rgb_file).astype(np.float32) / 255.0
    # depth = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED).astype(np.float32) / s
    # rgb, depth = torch.from_numpy(rgb), torch.from_numpy(depth)
    rgb = rgb/255.0
    depth = depth/camera_info.scale
    rgb, depth = torch.from_numpy(rgb).float(), torch.from_numpy(depth).float()

    depth = torch.clamp(depth, 0, 1)
    if FLAGS.model == 'convnet_resnet101':
        depth = depth.unsqueeze(-1).repeat([1, 1, 3])
        rgbd = torch.cat([rgb, depth], dim=-1).unsqueeze(0)
    elif 'depth' in FLAGS.model:
        rgbd = depth.unsqueeze(-1).unsqueeze(0)
    else:
        rgbd = torch.cat([rgb, depth.unsqueeze(-1)], dim=-1).unsqueeze(0)
    
    rgbd = rgbd.permute(0, 3, 1, 2)
    rgbd = rgbd.to(device)

    net.eval()
    tic = time.time()
    with torch.no_grad():
        pred = net(rgbd)
    pred = pred.clamp(0, 1)
    toc = time.time()
    print('inference time:', toc - tic)

    heatmap = (pred[0, 0] * pred[0, 1]).cpu().unsqueeze(0).unsqueeze(0)
    
    k_size = 15
    kernel = uniform_kernel(k_size)
    kernel = torch.from_numpy(kernel).unsqueeze(0).unsqueeze(0)
    heatmap = F.conv2d(heatmap, kernel, padding=(kernel.shape[2] // 2, kernel.shape[3] // 2)).squeeze().numpy()
    heatmap *= env_mask

    suctions, idx0, idx1 = get_suction_from_heatmap(depth.numpy(), heatmap, camera_info)
    
    save_dir = os.path.join(SAVE_PATH, split, 'scene_{}'.format(scene_idx), camera, 'suction')
    os.makedirs(save_dir, exist_ok=True)
    suction_numpy_file = os.path.join(save_dir, '%04d.npz'%anno_idx)
    print('Saving:', suction_numpy_file)
    np.savez(suction_numpy_file, suctions)

    depth = depth.numpy()
    cloud = create_point_cloud_from_depth_image(depth=depth,camera=camera_info,organized=False)
    # 从NumPy数组创建点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(cloud)
    point_cloud.transform(hand_eye_m)
    point_cloud.colors = o3d.utility.Vector3dVector(rgb.reshape(-1,3))

    # mask = suctions[:,0]>0.1
    mask = suctions[:,0]>0.6
    mask = find_top_k_ids(suctions[:,0], top_k)
    if not mask.any():
        print("no suction poses found!")
        return 
    suction_masked = suctions[mask]
    suction_normals, suction_points = suction_masked[:,1:4], suction_masked[:,4:]
    suction_points = transform_points(suction_points, hand_eye_m) # suction_points_robot
    suction_normals = transform_normals(suction_normals, hand_eye_m) # suction_normals_robot
    suction_scores = suction_masked[:,0]

    suction_point_cloud = o3d.geometry.PointCloud()
    suction_point_cloud.points = o3d.utility.Vector3dVector(suction_points)
    
    end_points = suction_points + suction_normals*0.1
    start_points = suction_points

    lines = [[i, i+len(suction_points)] for i in range(len(suction_points))]

    all_points = np.vstack((start_points,end_points))

    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(all_points),lines=o3d.utility.Vector2iVector(lines))
    color_map = plt.get_cmap("hot")
    
    colors = np.array([color_map(score) for score in suction_scores])
    line_set.colors = o3d.utility.Vector3dVector(colors[:,:3])

    base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
    cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    cam_frame.transform(np.reshape(hand_eye_m, (4,4)))

    # 可视化点云
    o3d.visualization.draw_geometries([base_frame, cam_frame, point_cloud, suction_point_cloud, line_set], point_show_normal=False)

    if FLAGS.save_visu:
        rgb_img = rgbd[0].permute(1, 2, 0)[..., :3].cpu().numpy()
        rgb_img *= 255
        
        # predictions
        score = pred[0, 0].clamp(0, 1).cpu().numpy()
        center = pred[0, 1].clamp(0, 1).cpu().numpy()

        score *= 255
        center *= 255
        mix = heatmap * 255

        score_img = cv2.applyColorMap(score.astype(np.uint8), cv2.COLORMAP_RAINBOW)
        score_img = score_img * 0.5 + rgb_img * 0.5
        score_img = score_img.astype(np.uint8)
        score_img = Image.fromarray(score_img)

        center_img = cv2.applyColorMap(center.astype(np.uint8), cv2.COLORMAP_RAINBOW)
        center_img = center_img * 0.5 + rgb_img * 0.5
        center_img = center_img.astype(np.uint8)
        center_img = Image.fromarray(center_img)

        mix_img = cv2.applyColorMap(mix.astype(np.uint8), cv2.COLORMAP_RAINBOW)
        mix_img = mix_img * 0.5 + rgb_img * 0.5
        mix_img = mix_img.astype(np.uint8)
        mix_img = Image.fromarray(mix_img)

        score_dir = os.path.join(SAVE_PATH, split, 'scene_{}'.format(scene_idx), camera, 'visu')
        os.makedirs(score_dir, exist_ok=True)
        score_file = os.path.join(score_dir, '%04d_smoothness.png'%anno_idx)
        print('saving:', score_file)
        score_img.save(score_file)

        center_dir = os.path.join(SAVE_PATH, split, 'scene_{}'.format(scene_idx), camera, 'visu')
        os.makedirs(center_dir, exist_ok=True)
        center_file = os.path.join(center_dir, '%04d_center.png'%anno_idx)
        print('saving:', center_file)
        center_img.save(center_file)

        mix_dir = os.path.join(SAVE_PATH, split, 'scene_{}'.format(scene_idx), camera, 'visu')
        os.makedirs(mix_dir, exist_ok=True)
        mix_file = os.path.join(mix_dir, '%04d_mix.png'%anno_idx)
        print('saving:', mix_file)
        mix_img.save(mix_file)

        # sampled suctions
        sampled_img = np.zeros_like(heatmap)
        for i in range(suctions.shape[0]):
            drawGaussian(sampled_img, [idx1[i], idx0[i]], suctions[i, 0], 3)
        
        sampled_img *= 255
        sampled_img = cv2.applyColorMap(sampled_img.astype(np.uint8), cv2.COLORMAP_RAINBOW)
        sampled_img = sampled_img * 0.5 + rgb_img * 0.5
        sampled_img = sampled_img.astype(np.uint8)
        sampled_img = Image.fromarray(sampled_img)
        
        sampled_dir = os.path.join(SAVE_PATH, split, 'scene_{}'.format(scene_idx), camera, 'visu')
        os.makedirs(sampled_dir, exist_ok=True)
        sampled_file = os.path.join(sampled_dir, '%04d_sampled.png'%anno_idx)
        print('saving:', sampled_file)
        sampled_img.save(sampled_file) 

    return suction_points, suction_normals, suction_scores, point_cloud


def inference(scene_idx):

    # Initialize the library, if the library is not found, add the library path as argument
    pykinect.initialize_libraries()

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
    # device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_UNBINNED
    # print(device_config)

    # Start device
    device = pykinect.start_device(config=device_config)
    print(device.calibration) # 打印内参
    """
        Rgb Intrinsic parameters: 
        cx: 637.8362426757812
        cy: 363.8018798828125
        fx: 601.1693115234375
        fy: 600.8593139648438
        k1: 0.40146124362945557
        k2: -2.5951035022735596
        k3: 1.587603211402893
        k4: 0.28464314341545105
        k5: -2.420356512069702
        k6: 1.510811686515808
        codx: 0.0
        cody: 0.0
        p2: -0.00018864621233660728
        p1: 0.00040857898420654237
        metric_radius: 0.0
    """
    
    anno_idx=0
    num_anno_idx=10
    inputs_list = []
    while(anno_idx<num_anno_idx):
        
        move_robot_home()
        
        # Get capture
        capture = device.update()
        
        # Get the color image from the capture
        ret_color, color_image = capture.get_color_image()  # (720, 1280, 4) 0-255
        if not (ret_color):
            continue

        # Get the colored depth
        ret_depth, transformed_depth_image = capture.get_transformed_depth_image()  # (720, 1280, 4) 毫米单位深度
        if not (ret_depth):
            continue

        ret_colored_depth, transformed_colored_depth_image = capture.get_transformed_colored_depth_image()

        # depth = frames.get_depth_frame()
        # color = frames.get_color_frame()
        depth = transformed_depth_image # 720 1280 0-21401
        color = color_image[:,:,[2,1,0]] # 720 1280 0-256

        # rgb_file = os.path.join(dataset_root, 'scenes/scene_{:04d}/{}/rgb/{:04d}.png'.format(scene_idx, camera, anno_idx))
        # depth_file = os.path.join(dataset_root, 'scenes/scene_{:04d}/{}/depth/{:04d}.png'.format(scene_idx, camera, anno_idx))
        # # segmask_file = os.path.join(dataset_root, 'scenes/scene_{:04d}/kinect/label/{:04d}.png'.format(scene_idx, anno_idx))
        # meta_file = os.path.join(dataset_root, 'scenes/scene_{:04d}/{}/meta/{:04d}.mat'.format(scene_idx, camera, anno_idx))

        intrinsics = np.array([601.1693115234375 , 0.,         637.8362426757812,
                          0. ,        600.8593139648438, 363.8018798828125,
                          0.,           0.,           1.        ]).reshape((3,3))
        fx, fy = intrinsics[0,0], intrinsics[1,1]
        cx, cy = intrinsics[0,2], intrinsics[1,2]

        k1 = 0.40146124362945557
        k2 = -2.5951035022735596
        k3 = 1.587603211402893
        k4 = 0.28464314341545105
        k5 = -2.420356512069702
        k6 = 1.510811686515808
        p2 = -0.00018864621233660728
        p1 = 0.00040857898420654237

        dist_coeffs = np.array([k1, k2, p1, p2, k3])

        show_image(color)
        # undistorted_color = undistort_image(color_image, intrinsics, dist_coeffs)
        # show_image(undistorted_color)

        width = 1280
        height = 720
        scale = 1000.0
        camera_info = CameraInfo(width, height, fx, fy, cx, cy, scale)
        hand_eye_m = np.load("/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/hand_eye_result.npy")
        mask = np.load("/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/mask.npy")
        mask = np.load("/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/mask2.npy")
        mask = np.load("/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/mask3.npy")

        mask_3d = np.repeat(mask[:, :, np.newaxis], 3, axis=2)
        color = color*mask_3d

        suction_points, suction_normals, suction_scores, point_cloud = inference_one_view(camera_info, color, depth, scene_idx, anno_idx, hand_eye_m=hand_eye_m, env_mask=mask, top_k=1024)

        for suction_point, suction_normal, suction_score in zip(suction_points, suction_normals, suction_scores):
            is_acute_angle, is_above_z0_plane = analyze_transform_matrix(suction_normal, suction_point[2])
            print(is_acute_angle, is_above_z0_plane)
            if not (is_acute_angle and is_above_z0_plane):
                continue

            rotation_matrix = z_axis_to_rotation_matrix(-suction_normal)
            axis, angle = mat2axangle(rotation_matrix)
            new_tcp = np.concatenate((suction_point, axis[:3] * angle), axis=0)

            mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
            mesh_sphere.translate(suction_point)

            # 创建一个圆柱体
            height=0.2
            mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.01, height=height)

            # 计算旋转
            current_direction = np.array([0, 0, 1])  # 当前朝向
            axis = np.cross(current_direction, suction_normal)  # 旋转轴
            angle = np.arccos(np.dot(current_direction, suction_normal))  # 旋转角度

            # 使用 scipy 来创建旋转矩阵
            r = R.from_rotvec(axis * angle)
            rot_matrix = r.as_matrix()

            # 应用旋转
            mesh_cylinder.rotate(rot_matrix, center=(0, 0, 0))

            # 指定圆柱体的位置
            translation_vec = suction_point + (suction_normal * height / 2)
            mesh_cylinder.translate(translation_vec)
            mesh_cylinder.paint_uniform_color([0.5, 0., 0.])

            o3d.visualization.draw_geometries([point_cloud, mesh_cylinder], point_show_normal=False)
            user_input = input("please input 1/0 for execute or not")
            if user_input=="1":
                # import pdb; pdb.set_trace()
                # move_robot(new_tcp+np.array([0, 0, 0.4, 0, 0 , 0]))
                # move_robot(new_tcp+np.array([0, 0, 0.19, 0, 0 , 0]))
                move_robot(new_tcp, suction_normal)

                
                # record the experiment result
                user_input = input("please input 1/0 for success/fail")
            else:
                user_input=="0"
            
            # 存储有效输入
            if user_input in ["1", "0"]:
                inputs_list.append(user_input)
            else:
                print("invalid input! only 1 / 0")
            break
        
        anno_idx = anno_idx + 1
    
    logs = np.asarray(inputs_list)
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"log_{current_time}.txt"
    np.savetxt(file_name,logs)

if __name__ == "__main__":
    
    scene_name = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    scene_list = [scene_name]
    
    for scene_idx in scene_list:
        inference(scene_idx)
    

    rtde_c.stopScript()