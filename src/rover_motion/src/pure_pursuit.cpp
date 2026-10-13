
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <cstddef>
#include <limits>

#include "rover_motion/pure_pursuit.hpp"

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rover_motion
{

PurePursuit::PurePursuit()
: Node("pure_pursuit_motion_planner_node"),
  look_ahead_distance_(0.8),
  max_linear_velocity_(0.9),
  max_angular_velocity_(1.0)
{
  // TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters
  declare_parameter<double>(
    "look_ahead_distance", look_ahead_distance_);

  declare_parameter<double>(
    "max_linear_velocity", max_linear_velocity_);

  declare_parameter<double>(
    "max_angular_velocity", max_angular_velocity_);

  look_ahead_distance_ =
    get_parameter("look_ahead_distance").as_double();

  max_linear_velocity_ =
    get_parameter("max_linear_velocity").as_double();

  max_angular_velocity_ =
    get_parameter("max_angular_velocity").as_double();

  // Subscribers
  rclcpp::QoS path_qos(1);
  path_qos.transient_local();
  
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/zigzag/path",
      path_qos,
      std::bind(
          &PurePursuit::pathCallback,
          this,
          std::placeholders::_1));

  // Velocity command publisher
  cmd_pub_ =
    create_publisher<geometry_msgs::msg::Twist>("/key_vel", 10);

  // Look-ahead point visualization
  carrot_pub_ =
    create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pure_pursuit/carrot", 10);

  // Control loop: 10 Hz
  control_loop_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PurePursuit::controlLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "Pure Pursuit node started.");
}


// ============================================================
// Path Callback
// ============================================================

void PurePursuit::pathCallback(
  const nav_msgs::msg::Path::SharedPtr path)
{
  global_plan_ = *path;
  last_nearest_idx_ = 0;

  RCLCPP_INFO(
    get_logger(),
    "Received path with %zu poses. Frame: %s",
    global_plan_.poses.size(),
    global_plan_.header.frame_id.c_str());

  // Stop if an empty path was received
  if (global_plan_.poses.empty()) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_pub_->publish(stop_cmd);

    RCLCPP_WARN(get_logger(), "Received an empty path.");
  }
}


// ============================================================
// Main Control Loop
// ============================================================

void PurePursuit::controlLoop()
{
  if (global_plan_.poses.empty()) {
    return;
  }

  // Get robot pose in odom frame
  geometry_msgs::msg::TransformStamped robot_pose_transform;

  try {
    robot_pose_transform = tf_buffer_->lookupTransform(
      "odom",
      "base_footprint_ekf",
      tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Could not transform odom to base_footprint: %s",
      ex.what());

    // Stop if robot pose is unavailable
    geometry_msgs::msg::Twist stop_cmd;
    cmd_pub_->publish(stop_cmd);
    return;
  }

  // Convert the path to the odom frame if necessary
  if (!transformPlan(robot_pose_transform.header.frame_id)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Unable to transform the path.");

    geometry_msgs::msg::Twist stop_cmd;
    cmd_pub_->publish(stop_cmd);
    return;
  }

  // Current robot pose
  geometry_msgs::msg::PoseStamped robot_pose_stamped;

  robot_pose_stamped.header.frame_id =
    robot_pose_transform.header.frame_id;

  robot_pose_stamped.header.stamp =
    robot_pose_transform.header.stamp;

  robot_pose_stamped.pose.position.x =
    robot_pose_transform.transform.translation.x;

  robot_pose_stamped.pose.position.y =
    robot_pose_transform.transform.translation.y;

  robot_pose_stamped.pose.position.z =
    robot_pose_transform.transform.translation.z;

  robot_pose_stamped.pose.orientation =
    robot_pose_transform.transform.rotation;

  // Find look-ahead point
  auto carrot_pose = getCarrotPose(robot_pose_stamped);

  // Distance from robot to carrot
  const double dx =
    carrot_pose.pose.position.x -
    robot_pose_stamped.pose.position.x;

  const double dy =
    carrot_pose.pose.position.y -
    robot_pose_stamped.pose.position.y;

  const double distance = std::hypot(dx, dy);

  // Stop when carrot is reached
  if (distance <= 0.1) {
    RCLCPP_INFO(get_logger(), "Goal reached.");

    geometry_msgs::msg::Twist stop_cmd;
    cmd_pub_->publish(stop_cmd);

    global_plan_.poses.clear();
    return;
  }

  // Publish carrot in odom frame for visualization
  carrot_pub_->publish(carrot_pose);

  // ==========================================================
  // Transform carrot into robot frame
  // ==========================================================

  tf2::Transform robot_tf;
  tf2::Transform carrot_tf;

  tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
  tf2::fromMsg(carrot_pose.pose, carrot_tf);

  // Robot-frame carrot coordinates
  const tf2::Transform carrot_robot_tf =
    robot_tf.inverse() * carrot_tf;

  geometry_msgs::msg::Pose carrot_robot_pose;
    carrot_robot_pose.position.x = carrot_robot_tf.getOrigin().x();
    carrot_robot_pose.position.y = carrot_robot_tf.getOrigin().y();
    carrot_robot_pose.position.z = carrot_robot_tf.getOrigin().z();
    carrot_robot_pose.orientation =
        tf2::toMsg(carrot_robot_tf.getRotation());

  // Calculate curvature
  const double curvature =
    getCurvature(carrot_robot_pose);

  // ==========================================================
  // Calculate velocity command
  // ==========================================================

  geometry_msgs::msg::Twist cmd_vel;

// Conservative initial lateral-acceleration limit.
constexpr double max_lateral_acceleration = 0.4;

const double abs_curvature = std::abs(curvature);

double linear_velocity = max_linear_velocity_;

if (abs_curvature > 1e-3) {
    // Reduce speed as the curve gets tighter.
    linear_velocity = std::min(
        linear_velocity,
        std::sqrt(max_lateral_acceleration / abs_curvature));

    // Keep angular velocity within its configured limit.
    linear_velocity = std::min(
        linear_velocity,
        max_angular_velocity_ / abs_curvature);
}

cmd_vel.linear.x = linear_velocity;

cmd_vel.angular.z = std::max(
    -max_angular_velocity_,
    std::min(
        max_angular_velocity_,
        linear_velocity * curvature));

cmd_pub_->publish(cmd_vel);
}



// ============================================================
// Find Look-ahead Point
// ============================================================

geometry_msgs::msg::PoseStamped
PurePursuit::getCarrotPose(
    const geometry_msgs::msg::PoseStamped & robot_pose)
{
    const auto & poses = global_plan_.poses;

    if (poses.empty()) {
        return robot_pose;
    }

    const std::size_t n = poses.size();

    // Search locally from the previous path position.
    const std::size_t start = std::min(last_nearest_idx_, n - 1);
    const std::size_t end = std::min(n - 1, start + 100);

    std::size_t nearest = start;
    double nearest_distance = std::numeric_limits<double>::max();

    for (std::size_t i = start; i <= end; ++i) {
        const double dx =
            poses[i].pose.position.x - robot_pose.pose.position.x;
        const double dy =
            poses[i].pose.position.y - robot_pose.pose.position.y;

        const double distance = std::hypot(dx, dy);

        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest = i;
        }
    }

    last_nearest_idx_ = nearest;

    // Select the first point ahead that reaches the look-ahead distance.
    for (std::size_t i = nearest; i < n; ++i) {
        const double dx =
            poses[i].pose.position.x - robot_pose.pose.position.x;
        const double dy =
            poses[i].pose.position.y - robot_pose.pose.position.y;

        if (std::hypot(dx, dy) >= look_ahead_distance_) {
            return poses[i];
        }
    }

    return poses.back();
}


// ============================================================
// Calculate Curvature
// ============================================================

double PurePursuit::getCurvature(
  const geometry_msgs::msg::Pose & carrot_pose)
{
  const double x = carrot_pose.position.x;
  const double y = carrot_pose.position.y;

  const double distance_squared = x * x + y * y;

  if (distance_squared > 0.001) {
    return 2.0 * y / distance_squared;
  }

  return 0.0;
}


// ============================================================
// Transform Path
// ============================================================

bool PurePursuit::transformPlan(const std::string & frame)
{
  if (global_plan_.header.frame_id == frame) {
    return true;
  }

  if (global_plan_.header.frame_id.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Path frame_id is empty.");

    return false;
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    // Transform: original path frame -> target frame
    transform = tf_buffer_->lookupTransform(
      frame,
      global_plan_.header.frame_id,
      tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not transform path from '%s' to '%s': %s",
      global_plan_.header.frame_id.c_str(),
      frame.c_str(),
      ex.what());

    return false;
  }

  // Transform every pose safely
  for (auto & pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose;

    tf2::doTransform(
      pose,
      transformed_pose,
      transform);

    pose = transformed_pose;
  }

  global_plan_.header.frame_id = frame;

  return true;
}

}  // namespace rover_motion


// ============================================================
// Main
// ============================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<rover_motion::PurePursuit>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
