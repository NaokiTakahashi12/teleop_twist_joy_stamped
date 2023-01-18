
#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <teleop_twist_stamped_node_parameters.hpp>


namespace teleop_twist_stamped_joy
{
double convertJoyAxesToScalar(
  const sensor_msgs::msg::Joy &,
  const int axes_index,
  const double axes_scaler
);
}  // namespace teleop_twist_stamped_joy
