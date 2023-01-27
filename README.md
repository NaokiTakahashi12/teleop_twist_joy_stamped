# teleop_twist_stamped_joy

This is a ros2 node package that converts and publishes sensor_msgs/msg/Joy to geometry_msgs/msg/TwistStamped.

# Node

## teleop_twist_stamped_joy_node

### Subscribe topics

+ `~/joy` (sensor_msgs/msg/Joy)

### Publish topics

+ `~/cmd_vel_stamped` (geometry_msgs/msg/TwistStamped)

### Parameters

The parameters of this node are mostly compatible with teleop_twist_joy.

+ `twist_frame_id` (string, default_value: base_link)
+ `enable_button` (int, default_value: 10)
+ `enable_turbo_button` (int, default_value: 0)
+ `require_enable_button` (bool, default_value: true)
+ `axis_linear.x` (int, default_value: 1)
+ `axis_linear.y` (int, default_value: -1)
+ `axis_linear.z` (int, default_value: -1)
+ `scale_linear.x` (double, default_value: 1.0)
+ `scale_linear.y` (double, default_value: 1.0)
+ `scale_linear.z` (double, default_value: 1.0)
+ `scale_linear_turbo.x` (double, default_value: 1.0)
+ `scale_linear_turbo.y` (double, default_value: 1.0)
+ `scale_linear_turbo.z` (double, default_value: 1.0)
+ `axis_angular.roll` (int, default_value: -1)
+ `axis_angular.pitch` (int, default_value: -1)
+ `axis_angular.yaw` (int, default_value: -1)
+ `scale_angular.roll` (double, default_value: 0.4)
+ `scale_angular.pitch` (double, default_value: 0.4)
+ `scale_angular.yaw` (double, default_value: 0.4)
+ `scale_angular_turbo.roll` (double, default_value: 0.8)
+ `scale_angular_turbo.pitch` (double, default_value: 0.8)
+ `scale_angular_turbo.yaw` (double, default_value: 0.8)

# Installation

## Requirements

ros2 humble or higher

## Dependent packages

+ sensor_msgs
+ geometry_msgs
+ generate_parameter_library
+ joy
+ teleop_twist_joy

## Build

```shell
$ cd <your colcon workspace>/src
$ git clone https://github.com/NaokiTakahashi12/teleop_twist_stamped_joy.git
$ cd ..
$ colcon build
```

## Usage

```shell
$ ros2 run teleop_twist_stamped_joy teleop_twist_stamped_joy_node
```
