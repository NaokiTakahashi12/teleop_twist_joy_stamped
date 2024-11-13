// MIT License
//
// Copyright (c) 2023 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <teleop_twist_stamped_joy/converter.hpp>

#include <teleop_twist_stamped_node_parameters.hpp>


namespace teleop_twist_stamped_joy
{
class TeleopTwistStampedNode : public rclcpp::Node
{
public:
  explicit TeleopTwistStampedNode(const rclcpp::NodeOptions &);
  ~TeleopTwistStampedNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr);

  void convertJoyToTwist(
    geometry_msgs::msg::Twist &,
    const sensor_msgs::msg::Joy &
  );

  bool guardJoyToTwist(const sensor_msgs::msg::Joy &) const;

};

TeleopTwistStampedNode::TeleopTwistStampedNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("teleop_twist_stamped_joy", node_options)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start teleop_joy_stamped_node");

  param_listener_ = std::make_shared<ParamListener>(
    this->get_node_parameters_interface()
  );
  params_ = param_listener_->get_params();

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "~/joy",
    rclcpp::SensorDataQoS(),
    std::bind(
      &TeleopTwistStampedNode::joyCallback,
      this,
      std::placeholders::_1
    )
  );
  twist_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel_stamped",
    rclcpp::ServicesQoS()
  );
}

TeleopTwistStampedNode::~TeleopTwistStampedNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start teleop_joy_stamped_node");
}

void TeleopTwistStampedNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  if (guardJoyToTwist(*joy_msg)) {
    const bool pushed_enable_button = joy_msg->buttons[params_.enable_button] == 1;

    auto pub_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    pub_msg->header.stamp = this->get_clock()->now();
    pub_msg->header.frame_id = params_.twist_frame_id;

    if (!pushed_enable_button && params_.require_enable_button) {
      twist_stamped_publisher_->publish(std::move(pub_msg));
      return;
    }
    convertJoyToTwist(pub_msg->twist, *joy_msg);
    twist_stamped_publisher_->publish(std::move(pub_msg));
  }
}

void TeleopTwistStampedNode::convertJoyToTwist(
  geometry_msgs::msg::Twist & twist_msg,
  const sensor_msgs::msg::Joy & joy_msg
)
{
  const bool enable_turbo_mode = joy_msg.buttons[params_.enable_turbo_button] == 1;
  if (enable_turbo_mode) {
    twist_msg.linear.x = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_linear.x,
      params_.scale_linear_turbo.x
    );
    twist_msg.linear.y = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_linear.y,
      params_.scale_linear_turbo.y
    );
    twist_msg.linear.z = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_linear.z,
      params_.scale_linear_turbo.z
    );
    twist_msg.angular.x = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_angular.roll,
      params_.scale_angular_turbo.roll
    );
    twist_msg.angular.y = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_angular.pitch,
      params_.scale_angular_turbo.pitch
    );
    twist_msg.angular.z = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_angular.yaw,
      params_.scale_angular_turbo.yaw
    );
  } else {
    twist_msg.linear.x = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_linear.x,
      params_.scale_linear.x
    );
    twist_msg.linear.y = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_linear.y,
      params_.scale_linear.y
    );
    twist_msg.linear.z = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_linear.z,
      params_.scale_linear.z
    );
    twist_msg.angular.x = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_angular.roll,
      params_.scale_angular.roll
    );
    twist_msg.angular.y = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_angular.pitch,
      params_.scale_angular.pitch
    );
    twist_msg.angular.z = convertJoyAxesToScalar(
      joy_msg,
      params_.axis_angular.yaw,
      params_.scale_angular.yaw
    );
  }
}

bool TeleopTwistStampedNode::guardJoyToTwist(const sensor_msgs::msg::Joy & joy_msg) const
{
  if (static_cast<unsigned int>(params_.enable_button) > joy_msg.buttons.size()) {
    RCLCPP_WARN(this->get_logger(), "Not found enable_button button index");
    return false;
  }
  if (not twist_stamped_publisher_) {
    RCLCPP_INFO(this->get_logger(), "Not initialize TwistStamped publisher");
    return false;
  }
  return true;
}
}  // teleop_twist_stamped_joy

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_stamped_joy::TeleopTwistStampedNode)
