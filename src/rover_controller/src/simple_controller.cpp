#include "rover_controller/simple_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>


using std::placeholders::_1;


SimpleController::SimpleController(const std::string& name)
                                  : Node(name)
                                  , left_wheel_prev_pos_(0.0)
                                  , right_wheel_prev_pos_(0.0)
                                  , x_(0.0)
                                  , y_(0.0)
                                  , theta_(0.0)
                                  , is_first_callback_(true)
{
    // NOTE: these defaults are only used if the node is launched without
    // parameters being passed in (e.g. run standalone without the launch
    // file). The launch file overrides these with the real measured values.
    declare_parameter("wheel_radius", 0.1);
    declare_parameter("wheel_separation", 0.5767);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/rover_controller/cmd_vel", 10, std::bind(&SimpleController::velCallback, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rover_controller/odom", 10);

    speed_conversion_ << wheel_radius_/2, wheel_radius_/2, wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;
    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion_);

    // Fill the Odometry message with invariant parameters
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    // NOTE: fill these in with values matching rover_controller's
    // pose_covariance_diagonal / twist_covariance_diagonal if this odometry
    // feeds a sensor-fusion filter (e.g. EKF), otherwise downstream nodes
    // will assume perfect confidence in these readings.
    // odom_msg_.pose.covariance[0] = 0.001;
    // odom_msg_.pose.covariance[7] = 0.001;
    // odom_msg_.pose.covariance[35] = 0.01;
    // odom_msg_.twist.covariance[0] = 0.001;
    // odom_msg_.twist.covariance[7] = 0.001;
    // odom_msg_.twist.covariance[35] = 0.01;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    prev_time_ = get_clock()->now();
}


void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // Implements the differential kinematic model
    // Given v and w, calculate the velocities of the wheels
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

    double left_speed = wheel_speed.coeff(1);
    double right_speed = wheel_speed.coeff(0);

    // IMPORTANT: this order must match EXACTLY the order of joints listed
    // under simple_velocity_controller.joints in rover_controllers.yaml:
    //   - left_wheel_joint
    //   - left_encoder_joint
    //   - right_wheel_joint
    //   - right_encoder_joint
    // Both left joints share the same commanded speed (they're on the same
    // mechanical side), same for the right joints.
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(left_speed);   // left_wheel_joint
    wheel_speed_msg.data.push_back(left_speed);   // left_encoder_joint
    wheel_speed_msg.data.push_back(right_speed);  // right_wheel_joint
    wheel_speed_msg.data.push_back(right_speed);  // right_encoder_joint


    wheel_cmd_pub_->publish(wheel_speed_msg);
}


void SimpleController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    // Implements the inverse differential kinematic model
    // Given the position of the wheels, calculates their velocities
    // then calculates the velocity of the robot wrt the robot frame
    // and then converts it in the global frame and publishes the TF

    // Look up wheel joints by NAME instead of assuming a fixed index.
    // /joint_states includes every joint on the robot (all 4 drive wheels
    // plus gripper_joint), so a hardcoded index is not reliable and will
    // silently read the wrong joint if the publishing order ever changes.
    auto left_it = std::find(state.name.begin(), state.name.end(), "left_encoder_joint");
    auto right_it = std::find(state.name.begin(), state.name.end(), "right_encoder_joint");

    if (left_it == state.name.end() || right_it == state.name.end())
    {
        RCLCPP_WARN(get_logger(), "left_encoder_joint or right_encoder_joint not found in /joint_states, skipping this update.");
        return;
    }

    size_t left_idx = std::distance(state.name.begin(), left_it);
    size_t right_idx = std::distance(state.name.begin(), right_it);

    // On the very first callback there is no valid previous position yet,
    // so just initialize and skip computing a delta this time. Otherwise
    // the first computed dp would be a spurious jump from 0.0 to whatever
    // the wheel's actual starting position is.
    if (is_first_callback_)
    {
        left_wheel_prev_pos_ = state.position.at(left_idx);
        right_wheel_prev_pos_ = state.position.at(right_idx);
        prev_time_ = state.header.stamp;
        is_first_callback_ = false;
        return;
    }

    double dp_left = state.position.at(left_idx) - left_wheel_prev_pos_;
    double dp_right = state.position.at(right_idx) - right_wheel_prev_pos_;
    rclcpp::Time msg_time = state.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    // Actualize the prev pose for the next itheration
    left_wheel_prev_pos_ = state.position.at(left_idx);
    right_wheel_prev_pos_ = state.position.at(right_idx);
    prev_time_ = state.header.stamp;

    // Guard against a degenerate/zero dt (can happen on duplicate timestamps)
    double dt_seconds = dt.seconds();
    if (dt_seconds <= 0.0)
    {
        return;
    }

    // Calculate the rotational speed of each wheel
    double fi_left = dp_left / dt_seconds;
    double fi_right = dp_right / dt_seconds;

    // Calculate the linear and angular velocity
    double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
    double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    // Calculate the position increment
    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;
    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    // Compose and publish the odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;
    odom_pub_->publish(odom_msg_);

    // TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();
    transform_broadcaster_->sendTransform(transform_stamped_);
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleController>("simple_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}