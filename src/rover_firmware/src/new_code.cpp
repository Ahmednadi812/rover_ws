#include "rover_firmware/rover_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <iomanip>

namespace rover_firmware
{

// Must match the gripper_joint upper limit in the URDF (in radians), so
// that the position values reported here for the real robot fall on the
// same scale as what Gazebo reports in simulation. The real robot has no
// encoder on the gripper, so we can only report 3 discrete points (closed,
// midway/moving, open) rather than a true continuous angle.
constexpr double GRIPPER_OPEN_ANGLE_RAD = 2.967;
constexpr double GRIPPER_MID_ANGLE_RAD = GRIPPER_OPEN_ANGLE_RAD / 2.0;

roverInterface::roverInterface()
{
}


roverInterface::~roverInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("roverInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn roverInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("roverInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // resize(), not reserve(): guarantees these vectors always have real
  // elements matching info_.joints.size() before anyone takes an address
  // into them via export_state_interfaces()/export_command_interfaces().
  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);

  // Resolve joint indices BY NAME instead of assuming a fixed order in the
  // xacro file.
  right_wheel_idx_ = -1;
  left_wheel_idx_ = -1;
  gripper_idx_ = -1;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const std::string &name = info_.joints[i].name;
    if (name == "right_wheel_joint")
    {
      right_wheel_idx_ = static_cast<int>(i);
    }
    else if (name == "left_wheel_joint")
    {
      left_wheel_idx_ = static_cast<int>(i);
    }
    else if (name == "gripper_joint")
    {
      gripper_idx_ = static_cast<int>(i);
    }
  }

  if (right_wheel_idx_ == -1 || left_wheel_idx_ == -1)
  {
    RCLCPP_FATAL(rclcpp::get_logger("roverInterface"),
                 "right_wheel_joint / left_wheel_joint not found in <ros2_control> joints! Aborting");
    return CallbackReturn::FAILURE;
  }

  if (gripper_idx_ == -1)
  {
    RCLCPP_WARN(rclcpp::get_logger("roverInterface"),
                "gripper_joint not found in <ros2_control> joints - gripper control disabled.");
  }

  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> roverInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> roverInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn roverInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("roverInterface"), "Starting robot hardware ...");

  // fill(), not assignment from an initializer list: never reallocates,
  // so pointers already exported stay valid.
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("roverInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("roverInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn roverInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("roverInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("roverInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("roverInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type roverInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Serial protocol from Arduino, comma-separated tokens:
  //   r<p|n><value>  -> right wheel velocity (existing, unchanged)
  //   l<p|n><value>  -> left wheel velocity (existing, unchanged)
  //   g<state>       -> gripper status: 'o' = open (limit switch reached),
  //                      'c' = closed (limit switch reached), 'm' = moving
  // Example line: "rp12.34,lp10.00,go,"
  if(arduino_.IsDataAvailable())
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    arduino_.ReadLine(message);
    std::stringstream ss(message);
    std::string res;
    while(std::getline(ss, res, ','))
    {
      if (res.empty())
      {
        continue;
      }

      if(res.at(0) == 'r' || res.at(0) == 'l')
      {
        int multiplier = res.at(1) == 'p' ? 1 : -1;
        double value = std::stod(res.substr(2, res.size()));
        int idx = (res.at(0) == 'r') ? right_wheel_idx_ : left_wheel_idx_;
        velocity_states_.at(idx) = multiplier * value;
        position_states_.at(idx) += velocity_states_.at(idx) * dt;
      }
      else if(res.at(0) == 'g' && gripper_idx_ != -1 && res.size() > 1)
      {
        char state = res.at(1);
        if (state == 'o')
        {
          position_states_.at(gripper_idx_) = GRIPPER_OPEN_ANGLE_RAD;
          velocity_states_.at(gripper_idx_) = 0.0;
        }
        else if (state == 'c')
        {
          position_states_.at(gripper_idx_) = 0.0;
          velocity_states_.at(gripper_idx_) = 0.0;
        }
        else if (state == 'm')
        {
          position_states_.at(gripper_idx_) = GRIPPER_MID_ANGLE_RAD;
          velocity_states_.at(gripper_idx_) = 1.0;
        }
      }
    }
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}


namespace
{
std::string format_velocity_token(char letter, double value)
{
  char sign = value >= 0 ? 'p' : 'n';
  double abs_value = std::abs(value);
  std::string zero_pad = (abs_value < 10.0) ? "0" : "";

  std::stringstream ss;
  ss << letter << sign << std::fixed << std::setprecision(2) << zero_pad << abs_value;
  return ss.str();
}
}  // namespace


hardware_interface::return_type roverInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  std::stringstream message_stream;

  message_stream << format_velocity_token('r', velocity_commands_.at(right_wheel_idx_)) << ","
                  << format_velocity_token('l', velocity_commands_.at(left_wheel_idx_)) << ",";

  if (gripper_idx_ != -1)
  {
    // gripper_joint command is +1 (open), -1 (close), or 0 (stop/hold).
    // The Arduino owns the motor-stop / limit-switch / magnet logic; this
    // just forwards the requested direction.
    message_stream << format_velocity_token('g', velocity_commands_.at(gripper_idx_)) << ",";
  }

  try
  {
    arduino_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("roverInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace rover_firmware

PLUGINLIB_EXPORT_CLASS(rover_firmware::roverInterface, hardware_interface::SystemInterface)