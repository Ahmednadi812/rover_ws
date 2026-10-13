#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CornerPdFollower final : public rclcpp::Node
{
public:
  CornerPdFollower()
  : Node("corner_pd_follower")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint_ekf");
    path_topic_ = declare_parameter<std::string>("path_topic", "/zigzag/path");
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/key_vel");

    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 1.2);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 1.7);
    max_deceleration_ = declare_parameter<double>("max_deceleration", 0.8);
    kp_heading_ = declare_parameter<double>("kp_heading", 2.2);
    kd_heading_ = declare_parameter<double>("kd_heading", 0.06);
    kp_cross_track_ = declare_parameter<double>("kp_cross_track", 1.4);
    corner_threshold_deg_ = declare_parameter<double>("corner_threshold_deg", 35.0);
    corner_position_tolerance_ = declare_parameter<double>("corner_position_tolerance", 0.08);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.10);
    heading_tolerance_deg_ = declare_parameter<double>("heading_tolerance_deg", 3.0);
    turn_stable_cycles_required_ = declare_parameter<int>("turn_stable_cycles", 3);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS path_qos(rclcpp::KeepLast(1));
    path_qos.reliable();
    path_qos.transient_local();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, path_qos,
      std::bind(&CornerPdFollower::pathCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CornerPdFollower::controlLoop, this));

    last_cycle_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "Corner PD follower ready: path=%s, cmd=%s, base_frame=%s",
      path_topic_.c_str(), cmd_topic_.c_str(), base_frame_.c_str());
  }

private:
  enum class Mode { DRIVE_SEGMENT, TURN_IN_PLACE, FINISHED };

  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kEpsilon = 1e-6;

  static double wrapAngle(double angle)
  {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
  }

  static double yawFromQuaternion(double x, double y, double z, double w)
  {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  void publishStop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    global_path_ = *msg;
    corner_indices_.clear();
    segment_start_idx_ = 0;
    corner_position_ = 0;
    stable_turn_cycles_ = 0;
    previous_heading_error_ = 0.0;
    previous_error_valid_ = false;
    goal_reached_logged_ = false;
    mode_ = Mode::DRIVE_SEGMENT;

    const auto & poses = global_path_.poses;
    if (poses.empty()) {
      publishStop();
      RCLCPP_WARN(get_logger(), "Received an empty path; publishing stop.");
      return;
    }

    if (poses.size() == 1) {
      corner_indices_.push_back(0);
    } else {
      const double threshold = corner_threshold_deg_ * kPi / 180.0;
      // The zigzag path is densely interpolated. Keep only vertices where
      // the direction changes by more than the configured corner threshold.
      for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
        const auto & a = poses[i - 1].pose.position;
        const auto & b = poses[i].pose.position;
        const auto & c = poses[i + 1].pose.position;
        const double in_dx = b.x - a.x;
        const double in_dy = b.y - a.y;
        const double out_dx = c.x - b.x;
        const double out_dy = c.y - b.y;
        if (std::hypot(in_dx, in_dy) < kEpsilon ||
            std::hypot(out_dx, out_dy) < kEpsilon) {
          continue;
        }
        const double h_in = std::atan2(in_dy, in_dx);
        const double h_out = std::atan2(out_dy, out_dx);
        if (std::abs(wrapAngle(h_out - h_in)) >= threshold) {
          // Avoid duplicate corner indices in malformed paths.
          if (corner_indices_.empty() || i > corner_indices_.back()) {
            corner_indices_.push_back(i);
          }
        }
      }
      const std::size_t final_index = poses.size() - 1;
      if (corner_indices_.empty() || corner_indices_.back() != final_index) {
        corner_indices_.push_back(final_index);
      }
    }

    finished_ = false;
    RCLCPP_INFO(
      get_logger(), "Received path: %zu poses, %zu detected corners, frame='%s'.",
      poses.size(), corner_indices_.size() > 0 ? corner_indices_.size() - 1 : 0,
      global_path_.header.frame_id.c_str());
  }

  bool getRobotPose(double & x, double & y, double & yaw)
  {
    if (global_path_.header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Path has an empty frame_id.");
      return false;
    }

    try {
      // Returns the robot pose expressed in the path's frame (usually map).
      const auto tf = tf_buffer_->lookupTransform(
        global_path_.header.frame_id, base_frame_, tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      yaw = yawFromQuaternion(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform %s into path frame '%s': %s",
        base_frame_.c_str(), global_path_.header.frame_id.c_str(), ex.what());
      return false;
    }
  }

  void controlLoop()
  {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_cycle_time_).count();
    last_cycle_time_ = now;
    dt = std::clamp(dt, 0.001, 0.2);

    if (global_path_.poses.empty() || corner_indices_.empty() || finished_) {
      return;
    }

    double rx = 0.0, ry = 0.0, robot_yaw = 0.0;
    if (!getRobotPose(rx, ry, robot_yaw)) {
      publishStop();
      return;
    }

    if (corner_position_ >= corner_indices_.size()) {
      finished_ = true;
      publishStop();
      return;
    }

    const std::size_t target_idx = corner_indices_[corner_position_];
    const auto & target_position = global_path_.poses[target_idx].pose.position;
    const double dx_target = target_position.x - rx;
    const double dy_target = target_position.y - ry;
    const double distance_to_target = std::hypot(dx_target, dy_target);
    const bool is_final_target = (target_idx == global_path_.poses.size() - 1);

    geometry_msgs::msg::PoseStamped target_msg = global_path_.poses[target_idx];
    target_pub_->publish(target_msg);

    if (mode_ == Mode::DRIVE_SEGMENT) {
      const double tolerance = is_final_target ? goal_tolerance_ : corner_position_tolerance_;
      if (distance_to_target <= tolerance) {
        if (is_final_target) {
          finished_ = true;
          mode_ = Mode::FINISHED;
          publishStop();
          if (!goal_reached_logged_) {
            RCLCPP_INFO(get_logger(), "Final path point reached; stopping.");
            goal_reached_logged_ = true;
          }
          return;
        }

        if (corner_position_ + 1 >= corner_indices_.size()) {
          finished_ = true;
          publishStop();
          return;
        }

        // Stop translation at the corner, then rotate to the next segment.
        const std::size_t next_idx = corner_indices_[corner_position_ + 1];
        const auto & next_position = global_path_.poses[next_idx].pose.position;
        turn_target_yaw_ = std::atan2(
          next_position.y - target_position.y,
          next_position.x - target_position.x);
        mode_ = Mode::TURN_IN_PLACE;
        stable_turn_cycles_ = 0;
        previous_error_valid_ = false;
        publishStop();
        RCLCPP_INFO(
          get_logger(), "Corner %zu reached; turning in place to %.1f deg.",
          corner_position_ + 1, turn_target_yaw_ * 180.0 / kPi);
        return;
      }

      // Follow the current straight segment and correct cross-track error.
      const auto & start_position =
        global_path_.poses[segment_start_idx_].pose.position;
      const double line_dx = target_position.x - start_position.x;
      const double line_dy = target_position.y - start_position.y;
      const double line_length = std::hypot(line_dx, line_dy);
      if (line_length < kEpsilon) {
        publishStop();
        return;
      }
      const double line_heading = std::atan2(line_dy, line_dx);
      const double from_start_x = rx - start_position.x;
      const double from_start_y = ry - start_position.y;
      const double cross_track_error =
        -std::sin(line_heading) * from_start_x +
         std::cos(line_heading) * from_start_y;

      // Braking-distance profile: slow down before each corner.
      const double stopping_speed = std::sqrt(
        std::max(0.0, 2.0 * max_deceleration_ *
        std::max(0.0, distance_to_target - corner_position_tolerance_)));
      double linear_speed = std::min(max_linear_velocity_, stopping_speed);

      const double desired_heading = line_heading - std::atan2(
        kp_cross_track_ * cross_track_error,
        std::max(linear_speed, 0.12));
      const double heading_error = wrapAngle(desired_heading - robot_yaw);
      const double angular = headingControl(heading_error, dt);

      // Reduce forward speed when heading is substantially misaligned.
      linear_speed *= std::max(0.15, std::cos(
        std::min(std::abs(heading_error), kPi / 2.0)));

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = std::max(0.0, linear_speed);
      cmd.angular.z = angular;
      cmd_pub_->publish(cmd);
      return;
    }

    if (mode_ == Mode::TURN_IN_PLACE) {
      const double heading_error = wrapAngle(turn_target_yaw_ - robot_yaw);
      const double angular = headingControl(heading_error, dt);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;  // Differential-drive in-place 90-degree turn.
      cmd.angular.z = angular;
      cmd_pub_->publish(cmd);

      const double tolerance = heading_tolerance_deg_ * kPi / 180.0;
      if (std::abs(heading_error) <= tolerance) {
        ++stable_turn_cycles_;
      } else {
        stable_turn_cycles_ = 0;
      }

      if (stable_turn_cycles_ >= std::max(1, turn_stable_cycles_required_)) {
        // The corner just reached becomes the start of the next straight segment.
        segment_start_idx_ = corner_indices_[corner_position_];
        ++corner_position_;
        mode_ = Mode::DRIVE_SEGMENT;
        stable_turn_cycles_ = 0;
        previous_error_valid_ = false;
        publishStop();
        RCLCPP_INFO(get_logger(), "Turn completed; following next segment.");
      }
    }
  }

  double headingControl(double heading_error, double dt)
  {
    double derivative = 0.0;
    if (previous_error_valid_) {
      derivative = wrapAngle(heading_error - previous_heading_error_) / dt;
    }
    previous_heading_error_ = heading_error;
    previous_error_valid_ = true;
    const double output = kp_heading_ * heading_error + kd_heading_ * derivative;
    return std::clamp(output, -max_angular_velocity_, max_angular_velocity_);
  }

  std::string base_frame_;
  std::string path_topic_;
  std::string cmd_topic_;
  double max_linear_velocity_{1.2};
  double max_angular_velocity_{1.7};
  double max_deceleration_{0.8};
  double kp_heading_{2.2};
  double kd_heading_{0.06};
  double kp_cross_track_{1.4};
  double corner_threshold_deg_{35.0};
  double corner_position_tolerance_{0.08};
  double goal_tolerance_{0.10};
  double heading_tolerance_deg_{3.0};
  int turn_stable_cycles_required_{3};

  nav_msgs::msg::Path global_path_;
  std::vector<std::size_t> corner_indices_;
  std::size_t segment_start_idx_{0};
  std::size_t corner_position_{0};
  Mode mode_{Mode::DRIVE_SEGMENT};
  double turn_target_yaw_{0.0};
  double previous_heading_error_{0.0};
  bool previous_error_valid_{false};
  int stable_turn_cycles_{0};
  bool finished_{false};
  bool goal_reached_logged_{false};
  std::chrono::steady_clock::time_point last_cycle_time_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CornerPdFollower>());
  rclcpp::shutdown();
  return 0;
}
