#include "rover_firmware/rover_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace rover_firmware
{

namespace
{
// IMPORTANT:
// These values follow the gripper URDF values you showed earlier:
// lower = -2.96706, upper = 0.0 rad.
// If your current URDF is different, change these three constants.
constexpr double GRIPPER_OPEN_ANGLE_RAD = -2.96706;
constexpr double GRIPPER_CLOSED_ANGLE_RAD = 0.0;
constexpr double GRIPPER_MID_ANGLE_RAD =
  (GRIPPER_OPEN_ANGLE_RAD + GRIPPER_CLOSED_ANGLE_RAD) / 2.0;

void trim_line_end(std::string & token)
{
  while (!token.empty() &&
         (token.back() == '\r' || token.back() == '\n' || token.back() == ' ' || token.back() == '\t'))
  {
    token.pop_back();
  }
}
}  // namespace

roverInterface::roverInterface()
{
}

roverInterface::~roverInterface()
{
  stop_ros_interfaces();

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      // Do not throw from a destructor.
    }
  }
}

int roverInterface::find_joint_index(const std::string & joint_name) const
{
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (info_.joints[i].name == joint_name)
    {
      return static_cast<int>(i);
    }
  }

  return -1;
}

CallbackReturn roverInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read serial port from ros2_control XML/YAML hardware parameters.
  const auto port_it = info_.hardware_parameters.find("port");
  if (port_it == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("rover_firmware"),
                 "Hardware parameter 'port' was not found.");
    return CallbackReturn::ERROR;
  }

  port_ = port_it->second;

  const std::size_t joint_count = info_.joints.size();
  velocity_commands_.assign(joint_count, 0.0);
  position_states_.assign(joint_count, 0.0);
  velocity_states_.assign(joint_count, 0.0);

  right_wheel_idx_ = find_joint_index("right_wheel_joint");
  left_wheel_idx_ = find_joint_index("left_wheel_joint");
  gripper_idx_ = find_joint_index("gripper_joint");

  // Keep the encoder joints exactly in the ros2_control hardware list.
  // We do not add a camera joint, so controller_manager will not get a new
  // camera command interface/mode just because of the camera servo.
  const int left_encoder_idx = find_joint_index("left_encoder_joint");
  const int right_encoder_idx = find_joint_index("right_encoder_joint");

  if (right_wheel_idx_ < 0 || left_wheel_idx_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rover_firmware"),
                 "right_wheel_joint and left_wheel_joint must exist.");
    return CallbackReturn::ERROR;
  }

  if (left_encoder_idx < 0 || right_encoder_idx < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                "Encoder joints were not found by name. The interface will still run.");
  }

  if (gripper_idx_ < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                "gripper_joint was not found. Gripper commands/status will be ignored.");
  }

  create_ros_interfaces();

  RCLCPP_INFO(rclcpp::get_logger("rover_firmware"),
              "Rover hardware initialized on port: %s", port_.c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> roverInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_states_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> roverInterface::export_command_interfaces()
{
  // Kept deliberately like the old code:
  // one velocity command interface for every joint listed by ros2_control.
  // This avoids changing the controller interface/mode because of the
  // encoder joints.
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_commands_[i]);
  }

  return command_interfaces;
}

CallbackReturn roverInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  camera_servo_command_.store(0.0);
  have_proximity_state_ = false;
  have_metal_detector_state_ = false;

  try
  {
    if (arduino_.IsOpen())
    {
      arduino_.Close();
    }

    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    RCLCPP_INFO(rclcpp::get_logger("rover_firmware"),
                "Arduino opened successfully: %s", port_.c_str());
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rover_firmware"),
                 "Failed to open Arduino serial port %s: %s",
                 port_.c_str(), ex.what());
    return CallbackReturn::ERROR;
  }

  // Start the internal ROS executor so the camera-servo subscriber can
  // receive joystick commands.
  if (ros_executor_ && !ros_interfaces_running_)
  {
    ros_executor_->add_node(ros_node_);
    ros_interfaces_running_ = true;
    ros_spin_thread_ = std::thread([this]() {
      ros_executor_->spin();
    });
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn roverInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  camera_servo_command_.store(0.0);

  // Send one stop frame before closing the serial port.
  if (arduino_.IsOpen())
  {
    try
    {
      const std::string stop_message =
        "rp00.00,lp00.00,gp00.00,sp00.00,\n";
      arduino_.Write(stop_message);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                  "Could not send stop command: %s", ex.what());
    }

    try
    {
      arduino_.Close();
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                  "Could not close Arduino port cleanly: %s", ex.what());
    }
  }

  stop_ros_interfaces();
  return CallbackReturn::SUCCESS;
}

std::string roverInterface::format_velocity_token(char letter, double value) const
{
  std::ostringstream ss;
  ss << letter << (value >= 0.0 ? 'p' : 'n')
     << std::fixed << std::setprecision(2) << std::abs(value);

  return ss.str();
}

hardware_interface::return_type roverInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  if (!arduino_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (!arduino_.IsDataAvailable())
  {
    return hardware_interface::return_type::OK;
  }

  std::string serial_data;

  try
  {
    arduino_.ReadLine(serial_data, '\n');
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                "Serial read failed: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }

  const double dt = period.seconds();

  std::stringstream ss(serial_data);
  std::string token;

  while (std::getline(ss, token, ','))
  {
    trim_line_end(token);

    if (token.empty())
    {
      continue;
    }

    // ------------------------------------------------------------
    // Right wheel velocity: rp12.34 / rn12.34
    // Left  wheel velocity: lp12.34 / ln12.34
    // ------------------------------------------------------------
    if (token.size() >= 3 &&
        (token[0] == 'r' || token[0] == 'l') &&
        (token[1] == 'p' || token[1] == 'n'))
    {
      try
      {
        double value = std::stod(token.substr(2));
        if (token[1] == 'n')
        {
          value = -value;
        }

        const int joint_idx = (token[0] == 'r') ? right_wheel_idx_ : left_wheel_idx_;

        if (joint_idx >= 0)
        {
          velocity_states_[joint_idx] = value;
          position_states_[joint_idx] += value * dt;
        }
      }
      catch (const std::exception &)
      {
        RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                    "Bad wheel token: %s", token.c_str());
      }

      continue;
    }

    // ------------------------------------------------------------
    // Proximity digital status: p1 / p0
    // ------------------------------------------------------------
    if (token.size() == 2 && token[0] == 'p' &&
        (token[1] == '0' || token[1] == '1'))
    {
      proximity_state_ = (token[1] == '1');
      have_proximity_state_ = true;

      if (proximity_pub_)
      {
        std_msgs::msg::Bool msg;
        msg.data = proximity_state_;
        proximity_pub_->publish(msg);
      }

      continue;
    }

    // ------------------------------------------------------------
    // Metal detector digital status: m1 / m0
    // ------------------------------------------------------------
    if (token.size() == 2 && token[0] == 'm' &&
        (token[1] == '0' || token[1] == '1'))
    {
      metal_detector_state_ = (token[1] == '1');
      have_metal_detector_state_ = true;

      if (metal_detector_pub_)
      {
        std_msgs::msg::Bool msg;
        msg.data = metal_detector_state_;
        metal_detector_pub_->publish(msg);
      }

      continue;
    }

    // ------------------------------------------------------------
    // Gripper status from Arduino:
    // go = open, gc = closed, gm = moving
    // ------------------------------------------------------------
    if (token.size() == 2 && token[0] == 'g' && gripper_idx_ >= 0)
    {
      switch (token[1])
      {
        case 'o':
          position_states_[gripper_idx_] = GRIPPER_OPEN_ANGLE_RAD;
          velocity_states_[gripper_idx_] = 0.0;
          break;

        case 'c':
          position_states_[gripper_idx_] = GRIPPER_CLOSED_ANGLE_RAD;
          velocity_states_[gripper_idx_] = 0.0;
          break;

        case 'm':
          position_states_[gripper_idx_] = GRIPPER_MID_ANGLE_RAD;
          velocity_states_[gripper_idx_] = 0.0;
          break;

        default:
          break;
      }

      continue;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type roverInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!arduino_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }

  std::ostringstream message;

  // Keep the wheel protocol unchanged.
  message << format_velocity_token('r', velocity_commands_[right_wheel_idx_]) << ',';
  message << format_velocity_token('l', velocity_commands_[left_wheel_idx_]) << ',';

  // Gripper still uses the ros2_control velocity command.
  if (gripper_idx_ >= 0)
  {
    message << format_velocity_token('g', velocity_commands_[gripper_idx_]) << ',';
  }

  // Camera servo is NOT a ros2_control joint.
  // The command comes from /camera_servo_cmd and is sent directly to Arduino.
  message << format_velocity_token('s', camera_servo_command_.load()) << ',';

  message << '\n';

  try
  {
    arduino_.Write(message.str());
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                "Serial write failed: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void roverInterface::create_ros_interfaces()
{
  // Avoid accidentally taking controller_manager command-line arguments.
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);

  ros_node_ = std::make_shared<rclcpp::Node>(
    "rover_hardware_interface", options);

  proximity_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    "/proximity", 10);

  metal_detector_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    "/metal_detector", 10);

  camera_servo_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
    "/camera_servo_cmd",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      // Expected values are +1.0, -1.0, or 0.0.
      double cmd = msg->data;

      if (cmd > 1.0)
      {
        cmd = 1.0;
      }
      else if (cmd < -1.0)
      {
        cmd = -1.0;
      }

      camera_servo_command_.store(cmd);
    });

  ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

void roverInterface::stop_ros_interfaces()
{
  if (ros_interfaces_running_ && ros_executor_)
  {
    ros_executor_->cancel();

    if (ros_spin_thread_.joinable())
    {
      ros_spin_thread_.join();
    }

    // It is safe to remove the node after the executor thread has stopped.
    try
    {
      ros_executor_->remove_node(ros_node_);
    }
    catch (...)
    {
      // Ignore cleanup errors.
    }

    ros_interfaces_running_ = false;
  }
}

}  // namespace rover_firmware

PLUGINLIB_EXPORT_CLASS(
  rover_firmware::roverInterface,
  hardware_interface::SystemInterface)