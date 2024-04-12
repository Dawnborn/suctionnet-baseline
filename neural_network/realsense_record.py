import pyrealsense2 as rs
import numpy as np
import cv2
import datetime

# 配置深度和彩色流
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# 开始流
pipeline.start(config)

# 获取当前日期和时间，格式化为字符串
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"video_{current_time}.mp4"

# 定义保存视频的编解码器和创建VideoWriter对象
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # 对于mp4格式
out = cv2.VideoWriter('{}.mp4'.format(filename), fourcc, 30.0, (1280, 720))

try:
    while True:
        # 等待一对连续的帧：深度和彩色
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 将图像转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())

        # 将视频帧写入文件
        out.write(color_image)

        # 显示彩色图像
        cv2.imshow('RealSense', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # 停止流
    pipeline.stop()
    out.release()
    cv2.destroyAllWindows()
