
#include <teleop_twist_stamped_joy/converter.hpp>


namespace teleop_twist_stamped_joy
{
double convertJoyAxesToScalar(
  const sensor_msgs::msg::Joy &joy_msg,
  const int axes_index,
  const double axes_scaler
)
{
  constexpr double failed_return = 0;

  if(static_cast<unsigned int>(axes_index) > joy_msg.axes.size())
  {
    return failed_return;
  }
  if(axes_index < 0)
  {
    return failed_return;
  }
  else
  {
    return axes_scaler * joy_msg.axes[axes_index];
  }
}
}  // namespace teleop_twist_stamped_joy
