import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # إرسال الـ initial pose
        self.send_initial_pose()

    def send_initial_pose(self):

        msg = PoseWithCovarianceStamped()

        # Frame
        msg.header.frame_id = "map"

        # Position
        msg.pose.pose.position.x = 2.0
        msg.pose.pose.position.y = 1.0
        msg.pose.pose.position.z = 0.0

        # Orientation
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Publish
        self.publisher.publish(msg)

        self.get_logger().info(
            'Initial pose sent: x=2.0, y=1.0, yaw=0.0'
        )


def main(args=None):
    rclpy.init(args=args)

    node = InitialPosePublisher()

    # نخلي الـ node شغال
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()