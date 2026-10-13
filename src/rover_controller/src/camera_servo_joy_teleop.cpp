#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

class CameraServoJoyTeleop : public rclcpp::Node
{
public:
  CameraServoJoyTeleop()
  : Node("camera_servo_joy_teleop")
  {
    left_button_ = declare_parameter<int>("left_button_index", 4);
    right_button_ = declare_parameter<int>("right_button_index", 1);
    command_value_ = declare_parameter<double>("command_value", 1.0);

    pub_ = create_publisher<std_msgs::msg::Float64>(
      "/camera_servo_cmd", 10);

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      rclcpp::QoS(10),
      std::bind(&CameraServoJoyTeleop::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Camera servo teleop started. left=%d right=%d",
                left_button_, right_button_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto button_pressed = [&msg](int index) -> bool {
      return index >= 0 &&
             static_cast<std::size_t>(index) < msg->buttons.size() &&
             msg->buttons[index] != 0;
    };

    const bool left_pressed = button_pressed(left_button_);
    const bool right_pressed = button_pressed(right_button_);

    double command = 0.0;

    if (left_pressed && !right_pressed)
    {
      command = +command_value_;
    }
    else if (right_pressed && !left_pressed)
    {
      command = -command_value_;
    }

    // Publish only when the command changes. Since joy_node keeps publishing,
    // a button release will generate a 0.0 message immediately.
    if (std::abs(command - last_command_) > 1e-9)
    {
      std_msgs::msg::Float64 out;
      out.data = command;
      pub_->publish(out);
      last_command_ = command;
    }
  }

  int left_button_;
  int right_button_;
  double command_value_;
  double last_command_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraServoJoyTeleop>());
  rclcpp::shutdown();
  return 0;
}