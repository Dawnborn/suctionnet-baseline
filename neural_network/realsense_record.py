import pyrealsense2 as rs
import numpy as np
import cv2
from datetime import datetime

# 配置深度和彩色流
pipeline = rs.pipeline()

# Configure streams
config = rs.config()
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
profile = pipeline.start(config)

device = profile.get_device()
depth_sensor = device.first_depth_sensor()
device.hardware_reset()

# 获取当前日期和时间，格式化为字符串
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"video_{current_time}.mp4"

# 定义保存视频的编解码器和创建VideoWriter对象
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # 对于mp4格式
out = cv2.VideoWriter('{}.mp4'.format(filename), fourcc, 30.0, (1280, 720))

pipeline = rs.pipeline()

# Configure streams
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

align_to = rs.stream.color
align = rs.align(align_to)

# Start streaming
cfg = pipeline.start(config)
while(True):

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    # depth = frames.get_depth_frame()
    # color = frames.get_color_frame()
    color = aligned_frames.get_color_frame() # 720 1280 3 0-256
    if not color:
        continue

    color = np.asanyarray(color.get_data())

    # 将视频帧写入文件
    color = color[:,:,[2,1,0]]
    out.write(color)

    # 显示彩色图像
    cv2.imshow('RealSense', color)
    if cv2.waitKey(1) & 0xFF == ord('p'):
        break

pipeline.stop()
out.release()
cv2.destroyAllWindows()
