import cv2

import pykinect_azure as pykinect

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

    while True:

        # Get capture
        capture = device.update()

        # Get the color image from the capture
        ret_color, color_image = capture.get_color_image()  # (720, 1280, 4) 0-255
        if not (ret_color):
            continue

        # Get the colored depth
        ret_depth, transformed_depth_image = capture.get_transformed_depth_image()  # (720,1280) 毫米单位深度
        if not (ret_depth):
            continue

        if not ret_color or not ret_depth:
            continue

        print(capture.calibration)
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

        # Overlay body segmentation on depth image
        cv2.imshow('Transformed Color Depth Image', color_image)

        # Press q key to stop
        if cv2.waitKey(1) == ord('q'):
            break
