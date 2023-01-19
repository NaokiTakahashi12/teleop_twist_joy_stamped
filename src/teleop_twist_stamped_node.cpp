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
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_subscription;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_twist_stamped_publisher;

  std::shared_ptr<ParamListener> m_param_listener;
  Params m_params;

  void joyCallback(const sensor_msgs::msg::Joy &);

  void convertJoyToTwist(
    geometry_msgs::msg::Twist &,
    const sensor_msgs::msg::Joy &
  );

  bool guardJoyToTwist(const sensor_msgs::msg::Joy &) const;

};

TeleopTwistStampedNode::TeleopTwistStampedNode(const rclcpp::NodeOptions &node_options)
: rclcpp::Node("teleop_twist_stamped_joy", node_options)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start teleop_joy_stamped_node");

  m_param_listener = std::make_shared<ParamListener>(
    this->get_node_parameters_interface()
  );
  m_params = m_param_listener->get_params();

  m_joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
    "~/joy",
    10,
    std::bind(
      &TeleopTwistStampedNode::joyCallback,
      this,
      std::placeholders::_1
    )
  );
  m_twist_stamped_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel_stamped",
    10
  );
}

TeleopTwistStampedNode::~TeleopTwistStampedNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start teleop_joy_stamped_node");
}

void TeleopTwistStampedNode::joyCallback(const sensor_msgs::msg::Joy &joy_msg)
{
  if(guardJoyToTwist(joy_msg))
  {
    const bool pushed_enable_button = joy_msg.buttons[m_params.enable_button] == 1;

    geometry_msgs::msg::TwistStamped pub_msg;

    pub_msg.header.stamp = this->get_clock()->now();
    pub_msg.header.frame_id = m_params.twist_frame_id;

    if(!pushed_enable_button && !m_params.require_enable_button)
    {
      m_twist_stamped_publisher->publish(pub_msg);
      return;
    }
    convertJoyToTwist(pub_msg.twist, joy_msg);
    m_twist_stamped_publisher->publish(pub_msg);
  }
}

void TeleopTwistStampedNode::convertJoyToTwist(
  geometry_msgs::msg::Twist &twist_msg,
  const sensor_msgs::msg::Joy &joy_msg
)
{
  const bool enable_turbo_mode = joy_msg.buttons[m_params.enable_turbo_button] == 1;
  if(enable_turbo_mode)
  {
    twist_msg.linear.x = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_linear.x,
      m_params.scale_linear_turbo.x
    );
    twist_msg.linear.y = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_linear.y,
      m_params.scale_linear_turbo.y
    );
    twist_msg.linear.z = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_linear.z,
      m_params.scale_linear_turbo.z
    );
    twist_msg.angular.x = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_angular.roll,
      m_params.scale_angular_turbo.roll
    );
    twist_msg.angular.y = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_angular.pitch,
      m_params.scale_angular_turbo.pitch
    );
    twist_msg.angular.z = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_angular.yaw,
      m_params.scale_angular_turbo.yaw
    );
  }
  else
  {
    twist_msg.linear.x = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_linear.x,
      m_params.scale_linear.x
    );
    twist_msg.linear.y = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_linear.y,
      m_params.scale_linear.y
    );
    twist_msg.linear.z = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_linear.z,
      m_params.scale_linear.z
    );
    twist_msg.angular.x = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_angular.roll,
      m_params.scale_angular.roll
    );
    twist_msg.angular.y = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_angular.pitch,
      m_params.scale_angular.pitch
    );
    twist_msg.angular.z = convertJoyAxesToScalar(
      joy_msg,
      m_params.axis_angular.yaw,
      m_params.scale_angular.yaw
    );
  }
}

bool TeleopTwistStampedNode::guardJoyToTwist(const sensor_msgs::msg::Joy &joy_msg) const
{
  if(static_cast<unsigned int>(m_params.enable_button) > joy_msg.buttons.size())
  {
    RCLCPP_WARN(this->get_logger(), "Not found enable_button button index");
    return false;
  }
  if(not m_twist_stamped_publisher)
  {
    RCLCPP_INFO(this->get_logger(), "Not initialize TwistStamped publisher");
    return false;
  }
  return true;
}
}  // teleop_twist_stamped_joy

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_stamped_joy::TeleopTwistStampedNode)
