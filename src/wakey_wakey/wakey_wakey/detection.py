import rclpy
from rclpy.node import Node

import numpy as np
try:
    import cv2
except ImportError:
    cv2 = None

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class DetectionNode(Node):
    """
    Detects user approach from a DepthAI image stream.

    Depth images are preferred because they measure the thing the project
    actually cares about: a user moving close to the robot. RGB color-marker
    detection and mono/RGB background subtraction are supported as fallbacks for
    demos when stereo depth is unreliable.
    """

    DETECTION_STATES = {'ALARMING', 'FLEEING'}

    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('image_topic', '/oak/stereo/image_raw')
        self.declare_parameter('detection_mode', 'auto')
        self.declare_parameter('difference_threshold', 35)
        self.declare_parameter('min_area_fraction', 0.02)
        self.declare_parameter('min_blob_pixels', 350)
        self.declare_parameter('background_alpha', 0.03)
        self.declare_parameter('publish_cooldown_sec', 1.0)
        self.declare_parameter('approach_distance_m', 1.2)
        self.declare_parameter('min_depth_mm', 200)
        self.declare_parameter('max_depth_mm', 4000)
        self.declare_parameter('depth_roi_top_fraction', 0.20)
        self.declare_parameter('depth_roi_bottom_fraction', 0.85)
        self.declare_parameter('debug_log_sec', 1.0)
        self.declare_parameter('hsv_lower_1', [0, 80, 50])
        self.declare_parameter('hsv_upper_1', [15, 255, 255])
        self.declare_parameter('hsv_lower_2', [165, 80, 50])
        self.declare_parameter('hsv_upper_2', [179, 255, 255])

        self.image_topic = self.get_parameter('image_topic').value
        self.detection_mode = self.get_parameter('detection_mode').value
        self.difference_threshold = int(self.get_parameter('difference_threshold').value)
        self.min_area_fraction = float(self.get_parameter('min_area_fraction').value)
        self.min_blob_pixels = int(self.get_parameter('min_blob_pixels').value)
        self.background_alpha = float(self.get_parameter('background_alpha').value)
        self.publish_cooldown_sec = float(self.get_parameter('publish_cooldown_sec').value)
        self.approach_distance_m = float(self.get_parameter('approach_distance_m').value)
        self.min_depth_mm = int(self.get_parameter('min_depth_mm').value)
        self.max_depth_mm = int(self.get_parameter('max_depth_mm').value)
        self.depth_roi_top_fraction = float(self.get_parameter('depth_roi_top_fraction').value)
        self.depth_roi_bottom_fraction = float(self.get_parameter('depth_roi_bottom_fraction').value)
        self.debug_log_sec = float(self.get_parameter('debug_log_sec').value)
        self.hsv_lower_1 = self.hsv_parameter('hsv_lower_1')
        self.hsv_upper_1 = self.hsv_parameter('hsv_upper_1')
        self.hsv_lower_2 = self.hsv_parameter('hsv_lower_2')
        self.hsv_upper_2 = self.hsv_parameter('hsv_upper_2')

        self.flee_trigger_pub = self.create_publisher(Bool, '/wakey/flee_trigger', 10)
        self.direction_pub = self.create_publisher(String, '/wakey/user_direction', 10)

        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(String, '/wakey/robot_state', self.on_state_change, 10)
        self.create_timer(2.0, self.image_watchdog)

        self.robot_state = 'IDLE'
        self.background = None
        self.last_publish_time = 0.0
        self.last_debug_time = 0.0
        self.last_image_time = 0.0
        self.last_image_encoding = None

        self.get_logger().info(
            f'Detection node listening for images on {self.image_topic} '
            f'in {self.detection_mode} mode.'
        )

        if self.detection_mode == 'color' and cv2 is None:
            self.get_logger().error('OpenCV is not available. Color detection cannot run.')

    def on_state_change(self, msg: String):
        self.robot_state = msg.data

        if self.robot_state == 'IDLE':
            self.last_publish_time = 0.0
        elif self.robot_state == 'ALARMING':
            self.get_logger().info('Alarm started. Waiting for user approach detection.')

    def on_image(self, msg: Image):
        self.last_image_time = self.get_clock().now().nanoseconds / 1e9

        if msg.encoding != self.last_image_encoding:
            self.last_image_encoding = msg.encoding
            self.get_logger().info(
                f'Receiving {msg.encoding} images from {self.image_topic} '
                f'at {msg.width}x{msg.height}.'
            )

        image, image_kind = self.image_msg_to_array(msg)

        if image is None:
            return

        if self.should_use_color(image_kind):
            detected, direction, foreground_info = self.detect_color_marker(image)
        elif image_kind == 'depth':
            detected, direction, foreground_info = self.detect_depth_approach(image)
        else:
            image = self.preprocess_intensity(image)
            self.update_background(image)

            if self.background is None:
                return

            detected, direction, foreground_info = self.detect_foreground(image)

        if self.robot_state not in self.DETECTION_STATES:
            return

        if not detected:
            self.log_detection_debug(direction, foreground_info)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_publish_time < self.publish_cooldown_sec:
            return

        self.last_publish_time = now
        self.publish_flee_trigger(True)
        self.publish_direction(direction)

        self.get_logger().info(
            f'User approach detected: direction={direction}, foreground={foreground_info}.'
        )

    def image_watchdog(self):
        now = self.get_clock().now().nanoseconds / 1e9
        publisher_count = self.count_publishers(self.image_topic)

        if self.last_image_time == 0.0:
            self.get_logger().warn(
                f'No images received yet from {self.image_topic}. '
                f'ROS publishers on this topic: {publisher_count}.'
            )
            return

        age = now - self.last_image_time
        if age > 2.0:
            self.get_logger().warn(
                f'Last image from {self.image_topic} was {age:.1f}s ago. '
                f'ROS publishers on this topic: {publisher_count}.'
            )

    def log_detection_debug(self, direction, foreground_info):
        if self.debug_log_sec <= 0.0:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_debug_time < self.debug_log_sec:
            return

        self.last_debug_time = now
        self.get_logger().info(
            f'No approach yet: direction={direction}, foreground={foreground_info}, '
            f'threshold={self.approach_distance_m:.2f}m.'
        )

    def hsv_parameter(self, name):
        value = self.get_parameter(name).value
        return np.array([int(part) for part in value], dtype=np.uint8)

    def image_msg_to_array(self, msg: Image):
        if msg.height == 0 or msg.width == 0:
            self.get_logger().warn('Received empty image.')
            return None, None

        if msg.encoding in ('16UC1', 'mono16'):
            image = np.frombuffer(msg.data, dtype=np.uint16)
            image = image.reshape((msg.height, msg.step // 2))[:, :msg.width]
            return image, 'depth'

        if msg.encoding == '32FC1':
            image = np.frombuffer(msg.data, dtype=np.float32)
            image = image.reshape((msg.height, msg.step // 4))[:, :msg.width]
            return image, 'depth'

        if msg.encoding in ('mono8', '8UC1'):
            image = np.frombuffer(msg.data, dtype=np.uint8)
            return image.reshape((msg.height, msg.step))[:, :msg.width].astype(np.float32), 'intensity'

        if msg.encoding in ('rgb8', 'bgr8'):
            row_width = msg.width * 3
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = image.reshape((msg.height, msg.step))[:, :row_width]
            image = image.reshape((msg.height, msg.width, 3))

            if self.detection_mode == 'color':
                if msg.encoding == 'rgb8':
                    image = image[:, :, ::-1]
                return image, 'color'

            return image.mean(axis=2).astype(np.float32), 'intensity'

        self.get_logger().warn(
            f'Unsupported image encoding {msg.encoding}. Expected 16UC1, 32FC1, mono8, 8UC1, rgb8, or bgr8.'
        )
        return None, None

    def should_use_color(self, image_kind):
        return image_kind == 'color' and self.detection_mode == 'color' and cv2 is not None

    def preprocess_intensity(self, image):
        if cv2 is None:
            return image

        return cv2.GaussianBlur(image.astype(np.uint8), (7, 7), 0).astype(np.float32)

    def update_background(self, image):
        if self.background is None or self.background.shape != image.shape:
            self.background = image.copy()
            return

        if self.robot_state == 'IDLE':
            self.background = (
                (1.0 - self.background_alpha) * self.background
                + self.background_alpha * image
            )

    def detect_foreground(self, image):
        diff = np.abs(image - self.background)
        mask = (diff >= self.difference_threshold).astype(np.uint8)

        if cv2 is not None:
            kernel = np.ones((5, 5), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        blob_pixels = int(mask.sum())
        area_fraction = blob_pixels / float(mask.size)

        detected = (
            area_fraction >= self.min_area_fraction
            and blob_pixels >= self.min_blob_pixels
        )

        if not detected:
            return False, 'center', area_fraction

        direction = self.direction_from_region_counts(mask)

        return True, direction, f'{area_fraction:.2%}'

    def detect_color_marker(self, bgr_image):
        if cv2 is None:
            return False, 'center', 'opencv unavailable'

        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask_1 = cv2.inRange(hsv, self.hsv_lower_1, self.hsv_upper_1)
        mask_2 = cv2.inRange(hsv, self.hsv_lower_2, self.hsv_upper_2)
        mask = (mask_1 > 0) | (mask_2 > 0)

        blob_pixels = int(mask.sum())
        area_fraction = blob_pixels / float(mask.size)
        detected = (
            area_fraction >= self.min_area_fraction
            and blob_pixels >= self.min_blob_pixels
        )

        if not detected:
            return False, 'center', f'color_area={area_fraction:.2%}'

        direction = self.direction_from_region_counts(mask)
        return True, direction, f'color_area={area_fraction:.2%}'

    def detect_depth_approach(self, depth_image):
        if depth_image.dtype.kind == 'f':
            depth_mm = depth_image * 1000.0
        else:
            depth_mm = depth_image

        h, w = depth_mm.shape
        y0 = int(h * self.depth_roi_top_fraction)
        y1 = int(h * self.depth_roi_bottom_fraction)
        roi = depth_mm[y0:y1, :]

        regions = {
            'left': roi[:, 0:w // 3],
            'center': roi[:, w // 3:2 * w // 3],
            'right': roi[:, 2 * w // 3:w],
        }

        region_distances = {}
        for name, region in regions.items():
            valid = region[
                (region >= self.min_depth_mm)
                & (region <= self.max_depth_mm)
            ]
            if valid.size:
                region_distances[name] = float(np.percentile(valid, 10)) / 1000.0

        if not region_distances:
            return False, 'center', 'no valid depth'

        direction = min(region_distances, key=region_distances.get)
        nearest_m = region_distances[direction]
        detected = nearest_m <= self.approach_distance_m

        return detected, direction, f'nearest={nearest_m:.2f}m'

    def direction_from_region_counts(self, mask):
        h, w = mask.shape
        y0 = int(h * self.depth_roi_top_fraction)
        y1 = int(h * self.depth_roi_bottom_fraction)
        roi = mask[y0:y1, :]

        regions = {
            'left': roi[:, 0:w // 3],
            'center': roi[:, w // 3:2 * w // 3],
            'right': roi[:, 2 * w // 3:w],
        }
        counts = {name: int(region.sum()) for name, region in regions.items()}
        direction = max(counts, key=counts.get)

        if counts[direction] == 0:
            return 'center'

        return direction

    def publish_flee_trigger(self, detected: bool):
        msg = Bool()
        msg.data = detected
        self.flee_trigger_pub.publish(msg)

    def publish_direction(self, direction: str):
        msg = String()
        msg.data = direction
        self.direction_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
