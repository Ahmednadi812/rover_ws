#ifndef ROVER_INTERFACE_HPP
#define ROVER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rover_firmware
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class roverInterface : public hardware_interface::SystemInterface
{
public:
  roverInterface();
  ~roverInterface() override;

  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  int find_joint_index(const std::string & joint_name) const;

  std::string format_velocity_token(char letter, double value) const;

  void create_ros_interfaces();
  void stop_ros_interfaces();

private:
  LibSerial::SerialPort arduino_;
  std::string port_;

  // One entry per joint in the ros2_control hardware_info order.
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // Joint indices. Encoders remain in the joint list exactly like the old code.
  int right_wheel_idx_{-1};
  int left_wheel_idx_{-1};
  int gripper_idx_{-1};

  // Camera servo is intentionally NOT a ros2_control joint.
  // It is controlled directly by the Arduino through /camera_servo_cmd.
  std::atomic<double> camera_servo_command_{0.0};

  // Digital sensor state.
  bool proximity_state_{false};
  bool metal_detector_state_{false};
  bool have_proximity_state_{false};
  bool have_metal_detector_state_{false};

  // A small ROS node owned by the hardware interface.
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proximity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr metal_detector_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr camera_servo_sub_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
  std::thread ros_spin_thread_;
  bool ros_interfaces_running_{false};
};

}  // namespace rover_firmware

#endif  // ROVER_INTERFACE_HPP