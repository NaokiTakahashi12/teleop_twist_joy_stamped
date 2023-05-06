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

#include <teleop_twist_stamped_joy/converter.hpp>


namespace teleop_twist_stamped_joy
{
double convertJoyAxesToScalar(
  const sensor_msgs::msg::Joy & joy_msg,
  const int axes_index,
  const double axes_scaler
)
{
  constexpr double failed_return = 0;

  if (static_cast<unsigned int>(axes_index) > joy_msg.axes.size()) {
    return failed_return;
  }
  if (axes_index < 0) {
    return failed_return;
  } else {
    return axes_scaler * joy_msg.axes[axes_index];
  }
}
}  // namespace teleop_twist_stamped_joy
