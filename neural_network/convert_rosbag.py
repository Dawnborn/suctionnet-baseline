import pyrealsense2 as rs
import cv2

# 打开bag文件并获取RGB图像流
pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file("/data/hdd1/storage/junpeng/downloads/20240411_155834.bag")
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipeline.start(config)

# 逐帧读取RGB图像并保存为图片
i = 0
while True:
    # 等待下一帧
    frames = pipeline.wait_for_frames()
    if not frames:
        break
    
    # 提取RGB图像
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue
    
    # 将RGB图像保存为图片
    img = np.asanyarray(color_frame.get_data())
    cv2.imwrite(f"img_{i}.png", img)
    i += 1

# 关闭管道
pipeline.stop()