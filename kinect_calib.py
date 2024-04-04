import numpy as np
import open3d as o3d
import cv2
 
def hand_eye_calib(points_cam, points_rob):
    cam = o3d.geometry.PointCloud()
    rob = o3d.geometry.PointCloud()
    cam.points = o3d.utility.Vector3dVector(points_cam)
    rob.points = o3d.utility.Vector3dVector(points_rob)
    cam.paint_uniform_color([1, 0, 0])
    rob.paint_uniform_color([0, 1, 0])
    list_id_cam = list(range(len(points_cam)))
    list_id_rob = list(range(len(points_rob)))
    assert (len(list_id_cam) >= 3 and len(list_id_rob) >= 3)
    assert (len(list_id_rob) == len(list_id_rob))
    corr = np.zeros((len(list_id_cam), 2))
    corr[:, 0] = list_id_cam
    corr[:, 1] = list_id_rob
    print("zhanglei:", corr)
    hand_eye_p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    final_hand_eye = hand_eye_p2p.compute_transformation(cam, rob, o3d.utility.Vector2iVector(corr))
    print(final_hand_eye)
 
    o3d.visualization.draw_geometries([cam.transform(final_hand_eye), rob])
    return final_hand_eye

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
    """
    depth is raw depth here, without being scaled by camera.scale
    """
    assert(depth.shape[0] == camera.height and depth.shape[1] == camera.width)
    xmap = np.arange(camera.width)
    ymap = np.arange(camera.height)
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depth/camera.scale
    points_x = (xmap - camera.cx) * points_z / camera.fx
    points_y = (ymap - camera.cy) * points_z / camera.fy
    cloud = np.stack([points_x, points_y, points_z], axis=-1)
    if not organized:
        cloud = cloud.reshape([-1, 3])
    return cloud
 
if __name__ == "__main__":
    # points_cam = np.array(
    #     [[-0.02075,  0.02597,  1.15615],
    #      [0.15487, 0.05592,   1.11818],
    #      [0.13379, -0.04346, 1.12400]] # camera 坐标系下的点
    # )
    points_cam = np.array(
        [[0.18999086222955386, -0.014402337594503479, 0.627],
        [0.1863014618523999, 0.1320305048139165, 0.554],
        [-0.11575900786613977, 0.05667892633017958, 0.606]]
    )

    # points_rob = np.array(
    #     [[0.2449664800249867, -0.36223072817512053, 0.005815013802759242],
    #      [0.42321621634623396, -0.3965202364977765, 0.0071650325018939726],
    #      [0.4039400827566261, -0.29704639533281213, 0.007148857394119079]]
    # )
    points_rob = np.array([[0.5315284341408161, -0.31606744270290527, 0.12170995566217216],
    [0.5290902012372901, -0.47521642506733847, 0.12233675564088844],
    [0.23767231568711195, -0.3911261471407533, 0.11825402441267664]]
    )
 
    final_hand_eye = hand_eye_calib(points_cam, points_rob)
 
    # cam
    # [[-0.25047815, - 0.13900883,  1.50225439],
    # [-0.02924966,  0.18728401,  1.46627832],
    # [-0.38933743,  0.22643211,  1.53078687]]
 
    # rob
    # [[-0.10405,-0.60642,0.01483],
    # [0.22889, -0.39027,0.01612],
    # [0.25768,-0.75808,0.01991]]
 
    # hand_eye_result
    # [[0.03031916,  0.99942402,  0.01524406,  0.0198895],
    #  [0.9824005,  -0.02698358, -0.18482733, -0.08615673],
    #  [-0.18430953, 0.02057958, -0.98265278,  1.44772014],
    #  [0.,          0.,          0.,          1.]]
    
    hand_eye_m = final_hand_eye
    print("hand_eye_result", hand_eye_m)
    np.save("hand_eye_result.npy", hand_eye_m)
    
    image = cv2.imread("/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/color2.png")
    depth = np.load("/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/depth2.npy")

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

    width = 1280
    height = 720
    scale = 1000.0
    camera_info = CameraInfo(width, height, fx, fy, cx, cy, scale)

    cloud = create_point_cloud_from_depth_image(depth, camera_info, organized=False)

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(cloud)
    point_cloud.colors = o3d.utility.Vector3dVector(image.reshape(-1,3)/255.)
 
    # frame base
    base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
    cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    cam_frame.transform(np.reshape(hand_eye_m, (4,4)))
    point_cloud.transform(np.reshape(hand_eye_m, (4,4)))
    o3d.visualization.draw_geometries([base_frame, cam_frame, point_cloud])