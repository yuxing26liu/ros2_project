#!/usr/bin/env python3
"""
ROS2 OAK-D background-subtraction test for Wakey Wakey.

This script subscribes to an existing ROS image topic, learns the first few
seconds as background, then reports foreground object area and direction.

Examples:
  python3 scripts/test_ros_oakd_background_detection.py
  python3 scripts/test_ros_oakd_background_detection.py --display
  python3 scripts/test_ros_oakd_background_detection.py --topic /oak/rgb/image_rect
"""

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
except ModuleNotFoundError:
    cv2 = None


class RosOakdBackgroundTest(Node):
    def __init__(self, args):
        super().__init__("ros_oakd_background_test")
        self.args = args

        self.background_accum = None
        self.background_frames = 0
        self.background = None
        self.start_time = time.monotonic()
        self.last_report = self.start_time
        self.frames = 0
        self.last_encoding = None

        self.create_subscription(Image, args.topic, self.on_image, 10)
        self.get_logger().info(
            f"Listening on {args.topic}. Keep the camera view still for "
            f"{args.learn_seconds:.1f}s while background is learned."
        )

    def on_image(self, msg):
        if cv2 is None:
            self.get_logger().error("OpenCV is unavailable. Install python3-opencv.")
            return

        frame = self.image_msg_to_bgr(msg)
        if frame is None:
            return

        now = time.monotonic()
        self.frames += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        if now - self.start_time < self.args.learn_seconds:
            if self.background_accum is None:
                self.background_accum = gray.astype(np.float32)
            else:
                self.background_accum += gray.astype(np.float32)
            self.background_frames += 1

            if now - self.last_report >= 0.5:
                remaining = self.args.learn_seconds - (now - self.start_time)
                self.get_logger().info(f"Learning background... {max(remaining, 0.0):.1f}s left")
                self.last_report = now
            return

        if self.background is None:
            if self.background_frames == 0:
                self.get_logger().error("No frames arrived during background learning.")
                return
            self.background = (self.background_accum / self.background_frames).astype(np.uint8)
            self.get_logger().info("Background learned. Move an object/person into view.")
            self.last_report = now

        mask = self.foreground_mask(gray)
        detected, direction, area_fraction, bbox = self.summarize_mask(mask)

        if now - self.last_report >= 0.5:
            fps = self.frames / (now - self.last_report)
            self.frames = 0
            self.last_report = now
            status = "DETECTED" if detected else "no object"
            self.get_logger().info(
                f"RGB OK | {fps:.1f} FPS | {status} | "
                f"direction={direction} area={area_fraction:.2%}"
            )

        if self.args.display:
            debug = self.draw_debug(frame, mask, detected, direction, area_fraction, bbox)
            cv2.imshow("ros-oakd-background-detection", debug)
            cv2.waitKey(1)

    def image_msg_to_bgr(self, msg):
        if msg.encoding != self.last_encoding:
            self.last_encoding = msg.encoding
            self.get_logger().info(
                f"Receiving {msg.encoding} images at {msg.width}x{msg.height}."
            )

        if msg.encoding in ("rgb8", "bgr8"):
            row_width = msg.width * 3
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = image.reshape((msg.height, msg.step))[:, :row_width]
            image = image.reshape((msg.height, msg.width, 3))
            if msg.encoding == "rgb8":
                image = image[:, :, ::-1]
            return image.copy()

        if msg.encoding in ("mono8", "8UC1"):
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = image.reshape((msg.height, msg.step))[:, :msg.width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        self.get_logger().warn(f"Unsupported encoding {msg.encoding}. Try /oak/rgb/image_raw.")
        return None

    def foreground_mask(self, gray):
        diff = cv2.absdiff(gray, self.background)
        _, mask = cv2.threshold(diff, self.args.threshold, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def summarize_mask(self, mask):
        h, w = mask.shape
        foreground_pixels = int(np.count_nonzero(mask))
        area_fraction = foreground_pixels / float(mask.size)

        if area_fraction < self.args.min_area_fraction:
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

    def draw_debug(self, frame, mask, detected, direction, area_fraction, bbox):
        debug = frame.copy()
        h, w = frame.shape[:2]
        cv2.line(debug, (w // 3, 0), (w // 3, h), (255, 255, 0), 1)
        cv2.line(debug, (2 * w // 3, 0), (2 * w // 3, h), (255, 255, 0), 1)

        if bbox is not None:
            x, y, box_w, box_h = bbox
            cv2.rectangle(debug, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)

        label = f"{'DETECTED' if detected else 'no object'} {direction} {area_fraction:.2%}"
        cv2.putText(debug, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return np.hstack((debug, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))


def main():
    parser = argparse.ArgumentParser(description="Test ROS OAK-D RGB background object detection.")
    parser.add_argument("--topic", default="/oak/rgb/image_raw", help="ROS image topic to subscribe to.")
    parser.add_argument("--learn-seconds", type=float, default=2.0)
    parser.add_argument("--threshold", type=int, default=30)
    parser.add_argument("--min-area-fraction", type=float, default=0.02)
    parser.add_argument("--display", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = RosOakdBackgroundTest(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
