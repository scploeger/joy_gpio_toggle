#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <wiringPi.h>

class JoyGpioToggle : public rclcpp::Node
{
public:
  JoyGpioToggle() : Node("joy_gpio_toggle"), gpio_state_(LOW), prev_button_state_(false)
  {
    // Declare parameters with defaults
    this->declare_parameter<int>("button", 0);
    this->declare_parameter<int>("gpio_pin", 17);

    // Get parameters from the parameter server (can be overridden by a YAML file)
    this->get_parameter("button", button_);
    this->get_parameter("gpio_pin", gpio_pin_);

    // Initialize wiringPi using BCM GPIO numbering
    if (wiringPiSetupGpio() == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize wiringPi");
      rclcpp::shutdown();
      return;
    }

    // Set the specified GPIO pin as an output and initialize it to LOW
    pinMode(gpio_pin_, OUTPUT);
    digitalWrite(gpio_pin_, gpio_state_);

    // Create subscription to the joy topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyGpioToggle::joy_callback, this, std::placeholders::_1));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Ensure the message contains the desired button index
    if (msg->buttons.size() <= static_cast<size_t>(button_)) {
      RCLCPP_WARN(this->get_logger(), "Joy message does not have button %d", button_);
      return;
    }

    // Get current state of the configured button (assumes 1 for pressed, 0 for released)
    bool current_button_state = (msg->buttons[button_] == 1);

    // Toggle on the rising edge (button transitions from released to pressed)
    if (current_button_state && !prev_button_state_) {
      gpio_state_ = (gpio_state_ == LOW ? HIGH : LOW);
      digitalWrite(gpio_pin_, gpio_state_);
      RCLCPP_INFO(this->get_logger(), "Toggled GPIO pin %d to %s", gpio_pin_,
                  (gpio_state_ == HIGH ? "HIGH" : "LOW"));
    }

    // Update the previous button state
    prev_button_state_ = current_button_state;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  int button_;
  int gpio_pin_;
  int gpio_state_;
  bool prev_button_state_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyGpioToggle>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
