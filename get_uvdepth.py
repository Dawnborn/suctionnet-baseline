import cv2
import matplotlib.pyplot as plt

import pykinect_azure as pykinect
import numpy as np

if __name__ == "__main__":

    # Initialize the library, if the library is not found, add the library path as argument
    pykinect.initialize_libraries()

    # Modify camera configuration
    device_config = pykinect.default_configuration
    device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
    # print(device_config)

    # Start device
    device = pykinect.start_device(config=device_config)

    print(device.calibration)
    """
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

    while True:

        # Get capture
        capture = device.update()

        # Get the color image from the capture
        ret_color, color_image = capture.get_color_image()  # (720, 1280, 4) 0-255
        if not (ret_color):
            continue

        # Get the colored depth
        ret_depth, transformed_depth_image = capture.get_transformed_depth_image()  # (720,1280) 毫米单位深度
        # ret_depth, transformed_depth_image = capture.get_transformed_colored_depth_image()  # (720,1280) 毫米单位深度
        if not (ret_depth):
            continue

        if not ret_color or not ret_depth:
            continue

        # 鼠标点击事件的回调函数
        def mouse_click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:  # 检查鼠标左键点击事件
                depth_value = transformed_depth_image[y, x]  # 获取点击位置的深度值
                print(f"Depth at position ({x}, {y}): {depth_value}")

        # 创建一个窗口
        cv2.namedWindow("RGB Image")

        # 设置鼠标回调函数
        cv2.setMouseCallback("RGB Image", mouse_click)

        # 显示RGB图像
        cv2.imshow("RGB Image", color_image)

        # plt.imsave("depth.png",transformed_depth_image)
        # np.save("depth2.npy",transformed_depth_image)
        # plt.imsave("color2.png",color_image)
        # break

        # Press q key to stop
        if cv2.waitKey(1) == ord('q'):
            break
