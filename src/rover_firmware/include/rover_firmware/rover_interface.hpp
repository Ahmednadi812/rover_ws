#ifndef ROVER_INTERFACE_HPP
#define ROVER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
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
  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int find_joint_index(const std::string & joint_name) const;
  std::string format_velocity_token(char letter, double value) const;
  void create_ros_interfaces();
  void stop_ros_interfaces();

  LibSerial::SerialPort arduino_;
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  int right_wheel_idx_{-1};
  int left_wheel_idx_{-1};
  int gripper_idx_{-1};
  int left_encoder_idx_{-1};
  int right_encoder_idx_{-1};
  double effective_counts_per_revolution_{2200.0};
  int64_t last_left_encoder_count_{0};
  int64_t last_right_encoder_count_{0};
  bool have_left_encoder_count_{false};
  bool have_right_encoder_count_{false};
  std::chrono::steady_clock::time_point last_left_encoder_time_{};
  std::chrono::steady_clock::time_point last_right_encoder_time_{};

  std::atomic<double> camera_servo_command_{0.0};
  std::atomic<bool> cycle_request_pending_{false};
  // 0=no command, 1=clear fault at lower limit, 2=operator-requested travel downward to lower limit.
  std::atomic<int> gripper_admin_command_pending_{0};

  bool proximity_state_{false};
  bool metal_detector_state_{false};
  bool have_proximity_state_{false};
  bool have_metal_detector_state_{false};
  bool ultrasonic_valid_state_{false};
  bool magnet_state_{false};
  bool siren_state_{false};

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proximity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr metal_detector_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ultrasonic_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr magnet_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr siren_state_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> proximity_sensor_pubs_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultrasonic_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_cycle_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_cycle_outcome_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_fault_reason_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr camera_servo_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cycle_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_recovery_sub_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
  std::thread ros_spin_thread_;
  bool ros_interfaces_running_{false};
};
}  // namespace rover_firmware

#endif  // ROVER_INTERFACE_HPP