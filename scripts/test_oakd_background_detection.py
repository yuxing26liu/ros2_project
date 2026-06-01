#!/usr/bin/env python3
"""
Standalone OAK-D background-subtraction test for Wakey Wakey.

This script does not use ROS. It opens the OAK-D RGB camera directly, learns a
background for a short warmup period, then reports whether a foreground object
is visible and which side of the image it is on.

Examples:
  python3 scripts/test_oakd_background_detection.py
  python3 scripts/test_oakd_background_detection.py --display
  python3 scripts/test_oakd_background_detection.py --learn-seconds 3 --threshold 30
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
    print("ERROR: Could not import cv2/OpenCV. Install it in this Python environment.")
    print("Try: python3 -m pip install opencv-python")
    sys.exit(1)


def create_pipeline():
    pipeline = dai.Pipeline()

    rgb = pipeline.create(dai.node.ColorCamera)
    rgb.setPreviewSize(640, 360)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb.setInterleaved(False)
    rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    rgb_out = pipeline.create(dai.node.XLinkOut)
    rgb_out.setStreamName("rgb")
    rgb.preview.link(rgb_out.input)

    return pipeline


def foreground_mask(frame, background, threshold):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    diff = cv2.absdiff(gray, background)
    _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def summarize_mask(mask, min_area_fraction):
    h, w = mask.shape
    foreground_pixels = int(np.count_nonzero(mask))
    area_fraction = foreground_pixels / float(mask.size)

    if area_fraction < min_area_fraction:
        return False, "center", area_fraction, None

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, "center", area_fraction, None

    contour = max(contours, key=cv2.contourArea)
    x, y, box_w, box_h = cv2.boundingRect(contour)
    centroid_x = x + box_w / 2.0

    if centroid_x < w / 3:
        direction = "left"
    elif centroid_x > 2 * w / 3:
        direction = "right"
    else:
        direction = "center"

    return True, direction, area_fraction, (x, y, box_w, box_h)


def draw_debug(frame, mask, detected, direction, area_fraction, bbox):
    debug = frame.copy()

    h, w = frame.shape[:2]
    cv2.line(debug, (w // 3, 0), (w // 3, h), (255, 255, 0), 1)
    cv2.line(debug, (2 * w // 3, 0), (2 * w // 3, h), (255, 255, 0), 1)

    if bbox is not None:
        x, y, box_w, box_h = bbox
        cv2.rectangle(debug, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)

    label = f"{'DETECTED' if detected else 'no object'} direction={direction} area={area_fraction:.2%}"
    cv2.putText(debug, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    return np.hstack((debug, mask_bgr))


def main():
    parser = argparse.ArgumentParser(description="Test OAK-D RGB background object detection.")
    parser.add_argument("--seconds", type=float, default=0.0, help="Stop after N seconds. 0 means run until Ctrl-C.")
    parser.add_argument("--learn-seconds", type=float, default=2.0, help="Seconds to learn the empty background.")
    parser.add_argument("--threshold", type=int, default=30, help="Pixel difference threshold.")
    parser.add_argument("--min-area-fraction", type=float, default=0.02, help="Minimum foreground area to count as object.")
    parser.add_argument("--display", action="store_true", help="Show RGB and foreground mask windows.")
    args = parser.parse_args()

    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("ERROR: No OAK/DepthAI device found.")
        print("Check USB cable, power, permissions, and that no other process is using the camera.")
        sys.exit(1)

    print(f"Found {len(devices)} DepthAI device(s).")
    print(f"Learning background for {args.learn_seconds:.1f}s. Keep the camera view still and empty.")

    pipeline = create_pipeline()

    with dai.Device(pipeline) as device:
        print(f"Connected. USB speed: {device.getUsbSpeed().name}")

        rgb_queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        start = time.monotonic()
        last_report = start
        frames = 0
        background_accum = None
        background_frames = 0
        background = None

        while True:
            now = time.monotonic()
            if args.seconds > 0 and now - start >= args.seconds:
                break

            rgb_msg = rgb_queue.tryGet()
            if rgb_msg is None:
                time.sleep(0.005)
                continue

            frame = rgb_msg.getCvFrame()
            frames += 1

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (7, 7), 0)

            if now - start < args.learn_seconds:
                if background_accum is None:
                    background_accum = gray.astype(np.float32)
                else:
                    background_accum += gray.astype(np.float32)
                background_frames += 1

                if now - last_report >= 0.5:
                    remaining = args.learn_seconds - (now - start)
                    print(f"Learning background... {max(remaining, 0.0):.1f}s left")
                    last_report = now
                continue

            if background is None:
                if background_frames == 0:
                    print("ERROR: No frames arrived while learning background.")
                    sys.exit(1)
                background = (background_accum / background_frames).astype(np.uint8)
                print("Background learned. Move an object/person into view.")
                last_report = now

            mask = foreground_mask(frame, background, args.threshold)
            detected, direction, area_fraction, bbox = summarize_mask(mask, args.min_area_fraction)

            if now - last_report >= 0.5:
                fps = frames / (now - last_report)
                frames = 0
                last_report = now
                status = "DETECTED" if detected else "no object"
                print(f"RGB OK | {fps:.1f} FPS | {status} | direction={direction} area={area_fraction:.2%}")

            if args.display:
                debug = draw_debug(frame, mask, detected, direction, area_fraction, bbox)
                cv2.imshow("oakd-background-detection", debug)
                if cv2.waitKey(1) == ord("q"):
                    break

            time.sleep(0.005)

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
