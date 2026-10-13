#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import String


class MineClassifier(Node):

    def __init__(self):
        super().__init__("mine_classifier")

        # آخر قراءات الحساسات
        self.metal_detected = False
        self.surface_detected = False

        # Subscribers
        self.create_subscription(
            Bool,
            "/metal_detected",
            self.metal_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/surface_detected",
            self.surface_callback,
            10
        )

    
        self.publisher = self.create_publisher(
            String,
            "/mine_type",
            10
        )

        self.last_type = ""

        self.get_logger().info("Mine Classifier Started")

    def metal_callback(self, msg):
        self.metal_detected = msg.data
        self.classify()

    def surface_callback(self, msg):
        self.surface_detected = msg.data
        self.classify()

    def classify(self):

        mine_type = ""

        
        if not self.metal_detected:
            return


        if self.surface_detected:
            mine_type = "Surface"


        else:
            mine_type = "Buried"

        if mine_type != self.last_type:

            out = String()
            out.data = mine_type

            self.publisher.publish(out)

            self.last_type = mine_type

            self.get_logger().info(f"Mine Type = {mine_type}")


def main(args=None):
    rclpy.init(args=args)

    node = MineClassifier()

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