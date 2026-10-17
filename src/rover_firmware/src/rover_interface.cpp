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
constexpr double GRIPPER_OPEN_ANGLE_RAD = 0.0;
constexpr double GRIPPER_CLOSED_ANGLE_RAD = -2.96706;
constexpr double GRIPPER_MID_ANGLE_RAD =
  (GRIPPER_OPEN_ANGLE_RAD + GRIPPER_CLOSED_ANGLE_RAD) / 2.0;
constexpr double DEFAULT_EFFECTIVE_COUNTS_PER_REV = 2200.0;
constexpr double TWO_PI = 6.2831853071795864769;
constexpr double RIGHT_ENCODER_SIGN = 1.0;
constexpr double LEFT_ENCODER_SIGN = -1.0;  // Firmware applies LEFT_ENCODER_INVERTED to velocity.

bool parseInt64Token(const std::string & token, const std::string & prefix, int64_t & value)
{
  if (token.rfind(prefix, 0) != 0 || token.size() <= prefix.size()) return false;
  try {
    std::size_t consumed = 0;
    const long long parsed = std::stoll(token.substr(prefix.size()), &consumed, 10);
    if (consumed != token.size() - prefix.size()) return false;
    value = static_cast<int64_t>(parsed);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

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
  left_encoder_idx_ = find_joint_index("left_encoder_joint");
  right_encoder_idx_ = find_joint_index("right_encoder_joint");
  const auto counts_it = info_.hardware_parameters.find("effective_counts_per_revolution");
  if (counts_it != info_.hardware_parameters.end()) {
    try {
      const double configured = std::stod(counts_it->second);
      if (std::isfinite(configured) && configured > 0.0) {
        effective_counts_per_revolution_ = configured;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                    "Invalid effective_counts_per_revolution; using default %.1f",
                    DEFAULT_EFFECTIVE_COUNTS_PER_REV);
        effective_counts_per_revolution_ = DEFAULT_EFFECTIVE_COUNTS_PER_REV;
      }
    } catch (const std::exception &) {
      RCLCPP_WARN(rclcpp::get_logger("rover_firmware"),
                  "Could not parse effective_counts_per_revolution; using default %.1f",
                  DEFAULT_EFFECTIVE_COUNTS_PER_REV);
      effective_counts_per_revolution_ = DEFAULT_EFFECTIVE_COUNTS_PER_REV;
    }
  }

  if (right_wheel_idx_ < 0 || left_wheel_idx_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rover_firmware"),
                 "right_wheel_joint and left_wheel_joint must exist.");
    return CallbackReturn::ERROR;
  }

  if (left_encoder_idx_ < 0 || right_encoder_idx_ < 0)
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
              "Rover hardware initialized on port: %s (effective encoder counts/rev %.1f). "
              "Confirm this calibration on the physical wheel/gearbox.",
              port_.c_str(), effective_counts_per_revolution_);

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
  ultrasonic_valid_state_ = false;
  magnet_state_ = false;
  siren_state_ = false;
  have_left_encoder_count_ = false;
  have_right_encoder_count_ = false;
  last_left_encoder_count_ = 0;
  last_right_encoder_count_ = 0;
  cycle_request_pending_.store(false);
  gripper_admin_command_pending_.store(0);

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
    // Cumulative quadrature counts: er:<count> / el:<count>.
    // Keep wheel-velocity feedback from rp/lp separate from encoder positions.
    // ------------------------------------------------------------
    int64_t count = 0;
    if (token.rfind("er:", 0) == 0 && right_encoder_idx_ >= 0) {
      if (!parseInt64Token(token, "er:", count)) {
        RCLCPP_WARN(rclcpp::get_logger("rover_firmware"), "Malformed right encoder token: %s", token.c_str());
        continue;
      }
      const double rad_per_count = TWO_PI / effective_counts_per_revolution_;
      position_states_[right_encoder_idx_] = static_cast<double>(count) * RIGHT_ENCODER_SIGN * rad_per_count;
      const auto sample_time = std::chrono::steady_clock::now();
      const double encoder_dt = have_right_encoder_count_
        ? std::chrono::duration<double>(sample_time - last_right_encoder_time_).count() : 0.0;
      if (have_right_encoder_count_ && std::isfinite(encoder_dt) && encoder_dt > 0.0 && encoder_dt <= 1.0) {
        const int64_t delta = count - last_right_encoder_count_;
        const double max_plausible = effective_counts_per_revolution_ * 20.0 * encoder_dt + 100.0;
        velocity_states_[right_encoder_idx_] = (std::abs(static_cast<double>(delta)) <= max_plausible)
          ? static_cast<double>(delta) * RIGHT_ENCODER_SIGN * rad_per_count / encoder_dt : 0.0;
      } else {
        velocity_states_[right_encoder_idx_] = 0.0;
      }
      last_right_encoder_count_ = count;
      last_right_encoder_time_ = sample_time;
      have_right_encoder_count_ = true;
      continue;
    }
    if (token.rfind("el:", 0) == 0 && left_encoder_idx_ >= 0) {
      if (!parseInt64Token(token, "el:", count)) {
        RCLCPP_WARN(rclcpp::get_logger("rover_firmware"), "Malformed left encoder token: %s", token.c_str());
        continue;
      }
      const double rad_per_count = TWO_PI / effective_counts_per_revolution_;
      position_states_[left_encoder_idx_] = static_cast<double>(count) * LEFT_ENCODER_SIGN * rad_per_count;
      const auto sample_time = std::chrono::steady_clock::now();
      const double encoder_dt = have_left_encoder_count_
        ? std::chrono::duration<double>(sample_time - last_left_encoder_time_).count() : 0.0;
      if (have_left_encoder_count_ && std::isfinite(encoder_dt) && encoder_dt > 0.0 && encoder_dt <= 1.0) {
        const int64_t delta = count - last_left_encoder_count_;
        const double max_plausible = effective_counts_per_revolution_ * 20.0 * encoder_dt + 100.0;
        velocity_states_[left_encoder_idx_] = (std::abs(static_cast<double>(delta)) <= max_plausible)
          ? static_cast<double>(delta) * LEFT_ENCODER_SIGN * rad_per_count / encoder_dt : 0.0;
      } else {
        velocity_states_[left_encoder_idx_] = 0.0;
      }
      last_left_encoder_count_ = count;
      last_left_encoder_time_ = sample_time;
      have_left_encoder_count_ = true;
      continue;
    }

    // Four individual raw proximity diagnostics, sent once per five telemetry frames.
    if (token.size() == 4 && token[0] == 'q' && token[1] >= '1' && token[1] <= '4' &&
        token[2] == ':' && (token[3] == '0' || token[3] == '1')) {
      const std::size_t sensor_index = static_cast<std::size_t>(token[1] - '1');
      if (sensor_index < proximity_sensor_pubs_.size() && proximity_sensor_pubs_[sensor_index]) {
        std_msgs::msg::Bool msg;
        msg.data = (token[3] == '1');
        proximity_sensor_pubs_[sensor_index]->publish(msg);
      }
      continue;
    }

    // Ultrasonic range (cm) and explicit validity are additive telemetry.
    if (token.rfind("us:", 0) == 0) {
      try {
        const double distance = std::stod(token.substr(3));
        if (std::isfinite(distance) && distance >= 0.0 && distance <= 25.0 && ultrasonic_distance_pub_) {
          std_msgs::msg::Float32 msg;
          msg.data = static_cast<float>(distance);
          ultrasonic_distance_pub_->publish(msg);
        }
      } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("rover_firmware"), "Malformed ultrasonic token: %s", token.c_str());
      }
      continue;
    }
    if (token == "uv:0" || token == "uv:1") {
      ultrasonic_valid_state_ = (token == "uv:1");
      if (ultrasonic_valid_pub_) {
        std_msgs::msg::Bool msg; msg.data = ultrasonic_valid_state_; ultrasonic_valid_pub_->publish(msg);
      }
      continue;
    }
    if (token.rfind("cs:", 0) == 0 && gripper_cycle_state_pub_) {
      std_msgs::msg::String msg; msg.data = token.substr(3); gripper_cycle_state_pub_->publish(msg);
      continue;
    }
    if (token.rfind("co:", 0) == 0 && gripper_cycle_outcome_pub_) {
      std_msgs::msg::String msg; msg.data = token.substr(3); gripper_cycle_outcome_pub_->publish(msg);
      continue;
    }
    if (token.rfind("fr:", 0) == 0 && gripper_fault_reason_pub_) {
      std_msgs::msg::String msg; msg.data = token.substr(3); gripper_fault_reason_pub_->publish(msg);
      continue;
    }
    if (token == "mg:0" || token == "mg:1") {
      magnet_state_ = (token == "mg:1");
      if (magnet_state_pub_) { std_msgs::msg::Bool msg; msg.data = magnet_state_; magnet_state_pub_->publish(msg); }
      continue;
    }
    if (token == "si:0" || token == "si:1") {
      siren_state_ = (token == "si:1");
      if (siren_state_pub_) { std_msgs::msg::Bool msg; msg.data = siren_state_; siren_state_pub_->publish(msg); }
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

  // Consume a cycle request exactly once. If serial transmission fails, requeue it.
  const bool include_cycle_request = cycle_request_pending_.exchange(false);
  if (include_cycle_request) message << "cycle:1,";
  const int admin_command = gripper_admin_command_pending_.exchange(0);
  if (admin_command == 1) message << "grip:reset,";
  else if (admin_command == 2) message << "grip:recover_down,";
  message << '\n';

  try
  {
    arduino_.Write(message.str());
  }
  catch (const std::exception & ex)
  {
    if (include_cycle_request) cycle_request_pending_.store(true);
    if (admin_command != 0) {
      int expected = 0;
      gripper_admin_command_pending_.compare_exchange_strong(expected, admin_command);
    }
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
  proximity_sensor_pubs_.clear();
  for (int sensor = 1; sensor <= 4; ++sensor) {
    proximity_sensor_pubs_.push_back(ros_node_->create_publisher<std_msgs::msg::Bool>(
      "/proximity/sensor_" + std::to_string(sensor), 10));
  }
  ultrasonic_distance_pub_ = ros_node_->create_publisher<std_msgs::msg::Float32>(
    "/ultrasonic/distance", 10);
  ultrasonic_valid_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    "/ultrasonic/valid", 10);
  gripper_cycle_state_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
    "/gripper/cycle_state", 10);
  gripper_cycle_outcome_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
    "/gripper/cycle_outcome", 10);
  gripper_fault_reason_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
    "/gripper/fault_reason", 10);
  magnet_state_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    "/magnet_state", 10);
  siren_state_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    "/siren_state", 10);

  cycle_request_sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
    "/gripper/cycle_request", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) cycle_request_pending_.store(true);
    });
  gripper_reset_sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
    "/gripper/reset_request", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) gripper_admin_command_pending_.store(1);
    });
  gripper_recovery_sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
    "/gripper/recovery_down_request", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) gripper_admin_command_pending_.store(2);
    });

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