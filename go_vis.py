import plotly.graph_objects as go
import cv2
import numpy as np

# 模拟简单的图像和深度图数据
# 真实应用中，image和depth将是实际的图像和深度图
image = cv2.imread("/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/color2.png")
depth = np.load("/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/depth2.npy")
depth = depth/1000.0

# 模拟的相机内参矩阵
intrinsics = np.array([601.1693115234375 , 0.,         637.8362426757812,
                    0. ,        600.8593139648438, 363.8018798828125,
                    0.,           0.,           1.        ]).reshape((3,3))
K = intrinsics

# 计算每个像素的XYZ坐标
height, width = depth.shape
X, Y = np.meshgrid(np.arange(width), np.arange(height))
Z = depth
X = (X - K[0, 2]) * Z / K[0, 0]
Y = (Y - K[1, 2]) * Z / K[1, 1]

# 将坐标和颜色值重塑为一维数组，以便于plotly处理
X = X.flatten()
Y = Y.flatten()
Z = Z.flatten()
colors = image.reshape(-1, 3) / 255.0  # 归一化颜色值

hand_eye_m = [[ 0.99948355, -0.00212897, -0.03206386,  0.36518713],
 [ 0.01229568, -0.89653382,  0.44280461, -0.60687769],
 [-0.02968905, -0.44297017, -0.89604464,  0.68282678],
 [ 0.,          0.,          0.,          1.,        ]]

kk=3

# 使用plotly创建三维散点图
fig = go.Figure(data=[go.Scatter3d(
    x=X[::kk], y=Y[::kk], z=Z[::kk],
    mode='markers',
    marker=dict(
        size=2,
        color=colors[::kk],                # 设置颜色
        opacity=0.8
    )
)])

# 设置图表布局
fig.update_layout(
    margin=dict(l=0, r=0, b=0, t=0),
    scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        zaxis_title='Z'
    )
)

fig.show()