#!/usr/bin/env python3
"""
Standalone OAK-D smoke test for Wakey Wakey.

This script does not use ROS. Run it on the robot/Pi to verify that:
  1. Python can import depthai.
  2. The OAK-D is visible over USB.
  3. RGB frames arrive.
  4. Stereo depth frames arrive, if the connected camera supports them.

Examples:
  python3 scripts/test_oakd_camera.py --seconds 10
  python3 scripts/test_oakd_camera.py --display
  python3 scripts/test_oakd_camera.py --no-depth
"""

import argparse
import sys
import time

try:
    import depthai as dai
except ModuleNotFoundError:
    print("ERROR: Could not import depthai. Install it in this Python environment.")
    print("Try: python3 -m pip install depthai")
    sys.exit(1)

try:
    import numpy as np
except ModuleNotFoundError:
    print("ERROR: Could not import numpy. Install it in this Python environment.")
    print("Try: python3 -m pip install numpy")
    sys.exit(1)

try:
    import cv2
except ModuleNotFoundError:
    cv2 = None


def set_board_socket(camera, socket):
    """DepthAI versions differ slightly; this keeps the script version-tolerant."""
    if hasattr(camera, "setBoardSocket"):
        camera.setBoardSocket(socket)
    else:
        camera.setCamera(str(socket).split(".")[-1].lower())


def create_pipeline(enable_depth):
    pipeline = dai.Pipeline()

    rgb = pipeline.create(dai.node.ColorCamera)
    rgb.setPreviewSize(640, 360)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb.setInterleaved(False)
    rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    rgb_out = pipeline.create(dai.node.XLinkOut)
    rgb_out.setStreamName("rgb")
    rgb.preview.link(rgb_out.input)

    if not enable_depth:
        return pipeline

    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    set_board_socket(left, dai.CameraBoardSocket.LEFT)
    set_board_socket(right, dai.CameraBoardSocket.RIGHT)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)

    try:
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    except Exception:
        pass

    left.out.link(stereo.left)
    right.out.link(stereo.right)

    depth_out = pipeline.create(dai.node.XLinkOut)
    depth_out.setStreamName("depth")
    stereo.depth.link(depth_out.input)

    return pipeline


def summarize_depth(depth_frame, min_mm=200, max_mm=4000):
    h, w = depth_frame.shape
    y0 = h // 3
    y1 = 2 * h // 3

    regions = {
        "left": depth_frame[y0:y1, 0:w // 3],
        "center": depth_frame[y0:y1, w // 3:2 * w // 3],
        "right": depth_frame[y0:y1, 2 * w // 3:w],
    }

    medians = {}
    for name, region in regions.items():
        valid = region[(region >= min_mm) & (region <= max_mm)]
        if valid.size:
            medians[name] = float(np.median(valid)) / 1000.0

    if not medians:
        return None, None

    direction = min(medians, key=medians.get)
    return medians[direction], direction


def maybe_show_frames(rgb_frame, depth_frame):
    if cv2 is None:
        print("OpenCV is unavailable, so --display cannot open preview windows.")
        return False

    cv2.imshow("oakd-rgb", rgb_frame)

    if depth_frame is not None:
        depth_8bit = cv2.normalize(depth_frame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depth_color = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_HOT)
        cv2.imshow("oakd-depth", depth_color)

    return cv2.waitKey(1) != ord("q")


def main():
    parser = argparse.ArgumentParser(description="Test an OAK-D camera with DepthAI.")
    parser.add_argument("--seconds", type=float, default=0.0, help="Stop after N seconds. 0 means run until Ctrl-C.")
    parser.add_argument("--display", action="store_true", help="Show RGB/depth preview windows with OpenCV.")
    parser.add_argument("--no-depth", action="store_true", help="Only test RGB, not stereo depth.")
    args = parser.parse_args()

    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("ERROR: No OAK/DepthAI device found.")
        print("Check USB cable, power, permissions, and that no other process is using the camera.")
        sys.exit(1)

    print(f"Found {len(devices)} DepthAI device(s):")
    for device_info in devices:
        print(f"  mxid={device_info.getMxId()} state={device_info.state.name}")

    pipeline = create_pipeline(enable_depth=not args.no_depth)

    with dai.Device(pipeline) as device:
        print(f"Connected. USB speed: {device.getUsbSpeed().name}")

        rgb_queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        depth_queue = None if args.no_depth else device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        start = time.monotonic()
        last_report = start
        frames = 0

        while True:
            now = time.monotonic()
            if args.seconds > 0 and now - start >= args.seconds:
                break

            rgb_msg = rgb_queue.tryGet()
            depth_msg = depth_queue.tryGet() if depth_queue is not None else None

            rgb_frame = rgb_msg.getCvFrame() if rgb_msg is not None else None
            depth_frame = depth_msg.getFrame() if depth_msg is not None else None

            if rgb_frame is not None:
                frames += 1

            if now - last_report >= 1.0:
                fps = frames / (now - last_report)
                frames = 0
                last_report = now

                if depth_frame is not None:
                    distance_m, direction = summarize_depth(depth_frame)
                    if distance_m is None:
                        print(f"RGB OK | {fps:.1f} FPS | depth frame OK | no valid center depth")
                    else:
                        print(f"RGB OK | {fps:.1f} FPS | nearest={distance_m:.2f}m direction={direction}")
                else:
                    depth_status = "disabled" if args.no_depth else "waiting"
                    print(f"RGB OK | {fps:.1f} FPS | depth={depth_status}")

            if args.display and rgb_frame is not None:
                keep_running = maybe_show_frames(rgb_frame, depth_frame)
                if not keep_running:
                    break

            time.sleep(0.005)

    if cv2 is not None:
        cv2.destroyAllWindows()

    print("Done.")


if __name__ == "__main__":
    main()
