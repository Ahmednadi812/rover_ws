#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraNode(Node):

    def __init__(self):

        super().__init__("camera_node")

        # Publisher
        self.image_publisher = self.create_publisher(
            Image,
            "/camera/captured_image",
            10
        )

        # Subscriber
        self.command_subscriber = self.create_subscription(
            String,
            "/capture_command",
            self.command_callback,
            10
        )

        self.bridge = CvBridge()

        # Open camera
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error(
                "Failed to open camera"
            )
            return

        self.current_frame = None

        # Continuously read camera
        self.timer = self.create_timer(
            0.03,
            self.camera_callback
        )

        self.get_logger().info(
            "Camera Node Started"
        )

        self.get_logger().info(
            "Waiting for command on /capture_command"
        )

    def camera_callback(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning(
                "Failed to read camera frame"
            )
            return

        # Save latest frame
        self.current_frame = frame.copy()

        # Show live camera
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

    def command_callback(self, msg):

        command = msg.data.strip().upper()

        self.get_logger().info(
            f"Received command: {command}"
        )

        # Ignore empty commands
        if command == "":
            return

        # Make sure camera has a frame
        if self.current_frame is None:

            self.get_logger().warning(
                "No camera frame available"
            )

            return

        # Capture current frame
        frame = self.current_frame.copy()

        # Convert OpenCV image to ROS Image
        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8"
        )

        # Publish captured image
        self.image_publisher.publish(
            image_msg
        )

        self.get_logger().info(
            f"Image captured because command '{command}' was received"
        )

    def destroy_node(self):

        if self.cap.isOpened():
            self.cap.release()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()