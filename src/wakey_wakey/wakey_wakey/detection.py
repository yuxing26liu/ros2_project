import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class DetectionNode(Node):
    """
    Detects user approach from a black-and-white DepthAI image stream.

    The node learns a background while IDLE, freezes it when ALARMING starts,
    then publishes a flee trigger and approach direction when enough of the
    image changes.
    """

    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('image_topic', '/oak/right/image_rect')
        self.declare_parameter('difference_threshold', 35)
        self.declare_parameter('min_area_fraction', 0.08)
        self.declare_parameter('min_blob_pixels', 350)
        self.declare_parameter('left_boundary_fraction', 0.40)
        self.declare_parameter('right_boundary_fraction', 0.60)
        self.declare_parameter('background_alpha', 0.03)
        self.declare_parameter('publish_cooldown_sec', 1.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.difference_threshold = int(self.get_parameter('difference_threshold').value)
        self.min_area_fraction = float(self.get_parameter('min_area_fraction').value)
        self.min_blob_pixels = int(self.get_parameter('min_blob_pixels').value)
        self.left_boundary_fraction = float(self.get_parameter('left_boundary_fraction').value)
        self.right_boundary_fraction = float(self.get_parameter('right_boundary_fraction').value)
        self.background_alpha = float(self.get_parameter('background_alpha').value)
        self.publish_cooldown_sec = float(self.get_parameter('publish_cooldown_sec').value)

        self.flee_trigger_pub = self.create_publisher(Bool, '/wakey/flee_trigger', 10)
        self.direction_pub = self.create_publisher(String, '/wakey/user_direction', 10)

        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(String, '/wakey/robot_state', self.on_state_change, 10)

        self.robot_state = 'IDLE'
        self.background = None
        self.last_publish_time = 0.0

        self.get_logger().info(
            f'Detection node listening for mono images on {self.image_topic}.'
        )

    def on_state_change(self, msg: String):
        self.robot_state = msg.data

        if self.robot_state == 'IDLE':
            self.last_publish_time = 0.0

    def on_image(self, msg: Image):
        image = self.image_msg_to_grayscale(msg)

        if image is None:
            return

        self.update_background(image)

        if self.robot_state != 'ALARMING' or self.background is None:
            return

        detected, direction, area_fraction = self.detect_foreground(image)

        if not detected:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_publish_time < self.publish_cooldown_sec:
            return

        self.last_publish_time = now
        self.publish_direction(direction)
        self.publish_flee_trigger(True)

        self.get_logger().info(
            f'User approach detected: direction={direction}, foreground={area_fraction:.2%}.'
        )

    def image_msg_to_grayscale(self, msg: Image):
        if msg.height == 0 or msg.width == 0:
            self.get_logger().warn('Received empty image.')
            return None

        if msg.encoding in ('mono8', '8UC1'):
            image = np.frombuffer(msg.data, dtype=np.uint8)
            return image.reshape((msg.height, msg.step))[:, :msg.width].astype(np.float32)

        if msg.encoding in ('rgb8', 'bgr8'):
            row_width = msg.width * 3
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = image.reshape((msg.height, msg.step))[:, :row_width]
            image = image.reshape((msg.height, msg.width, 3))
            return image.mean(axis=2).astype(np.float32)

        self.get_logger().warn(
            f'Unsupported image encoding {msg.encoding}. Expected mono8, 8UC1, rgb8, or bgr8.'
        )
        return None

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
        mask = diff >= self.difference_threshold

        blob_pixels = int(mask.sum())
        area_fraction = blob_pixels / float(mask.size)

        detected = (
            area_fraction >= self.min_area_fraction
            and blob_pixels >= self.min_blob_pixels
        )

        if not detected:
            return False, 'center', area_fraction

        _, xs = np.nonzero(mask)
        centroid_x_fraction = float(xs.mean()) / float(mask.shape[1])

        if centroid_x_fraction < self.left_boundary_fraction:
            direction = 'left'
        elif centroid_x_fraction > self.right_boundary_fraction:
            direction = 'right'
        else:
            direction = 'center'

        return True, direction, area_fraction

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
