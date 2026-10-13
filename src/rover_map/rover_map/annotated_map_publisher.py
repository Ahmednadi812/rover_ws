import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from PIL import Image, ImageDraw

class AnnotatedMapNode(Node):
    def __init__(self):
        super().__init__('annotated_map_node')
        self.marker_id = 0

        # Declare parameters for map file paths and resolution
        self.declare_parameter('input_map_path', 'map.pgm')
        self.declare_parameter('output_map_path', 'annotated_map.png')
        self.declare_parameter('resolution', 0.05)  # meters per pixel
        self.declare_parameter('origin_x', -6.0)     # map origin X (from yaml)
        self.declare_parameter('origin_y', -6.0)     # map origin Y (from yaml)

        # Load parameters
        self.input_map_path = self.get_parameter('input_map_path').get_parameter_value().string_value
        self.output_map_path = self.get_parameter('output_map_path').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.origin_x = self.get_parameter('origin_x').get_parameter_value().double_value
        self.origin_y = self.get_parameter('origin_y').get_parameter_value().double_value

        # Load and convert initial map image to RGB
        if os.path.exists(self.input_map_path):
            self.map_img = Image.open(self.input_map_path).convert('RGB')
            self.get_logger().info(f'Loaded base map from {self.input_map_path}')
        else:
            self.get_logger().error(f'Input map file not found: {self.input_map_path}')
            raise FileNotFoundError(f'File not found: {self.input_map_path}')

        self.draw = ImageDraw.Draw(self.map_img)

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers for sensor topics
        self.create_subscription(Bool, 'sensor_1', self.sensor_1_callback, 10)
        self.create_subscription(Bool, 'sensor_2', self.sensor_2_callback, 10)

        # Publisher for RViz visualization
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

    def world_to_pixel(self, wx, wy):
        """Converts ROS world coordinates (meters) to Image pixel coordinates."""
        img_w, img_h = self.map_img.size
        px = int((wx - self.origin_x) / self.resolution)
        py = int(img_h - ((wy - self.origin_y) / self.resolution))  # Flip Y axis
        return px, py

    def sensor_1_callback(self, msg):
        if msg.data == 1:
            self.process_event(r=1.0, g=0.0, b=0.0, fill_color=(255, 0, 0))  # Red

    def sensor_2_callback(self, msg):
        if msg.data == 1:
            self.process_event(r=0.0, g=1.0, b=0.0, fill_color=(0, 255, 0))  # Green

    def process_event(self, r, g, b, fill_color):
        try:
            # Lookup robot pose in 'map' frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint_ekf',
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform map to base_link: {ex}')
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        # 1. Publish RViz 3D Marker
        self.publish_rviz_marker(x, y, transform.transform.translation.z, transform.transform.rotation, r, g, b)

        # 2. Draw Marker onto Map Image
        px, py = self.world_to_pixel(x, y)
        radius = 4  # Circle size on pixel image
        self.draw.ellipse([px - radius, py - radius, px + radius, py + radius], fill=fill_color, outline=(0, 0, 0))

        # 3. Save Updated Image
        self.map_img.save(self.output_map_path)
        self.get_logger().info(f'Marked pose [{x:.2f}, {y:.2f}] (Pixel: {px}, {py}) -> Saved to {self.output_map_path}')

    def publish_rviz_marker(self, x, y, z, orientation, r, g, b):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'sensor_events'
        marker.id = self.marker_id
        self.marker_id += 1

        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation = orientation

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.3

        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = AnnotatedMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()