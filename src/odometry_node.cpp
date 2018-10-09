/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 ThundeRatz

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <cmath>
#include <vector>
#include <iostream>

namespace ImuData
{
  double z_angle;
}

namespace RobotData
{
  double servo_angle;
}

namespace GPIO
{
  bool hall_sensor;
}

namespace
{
int is_gpio_stable(bool gpio, int expected_value)
{
  // FIXME Use a constant sampling rate
  for (int i = 0; i < 10; i++)
  {
    if (gpio != expected_value)
      return 0;
    ros::spinOnce();
  }
  return 1;
}
}  // namespace

// Servo angle subscriber
class ServoAngle
{
public:
  explicit ServoAngle(ros::NodeHandle node_handle)
  {
    servo_sub = node_handle.subscribe("/servo_angle", 10, &ServoAngle::servo_callback, this);
  }

  void servo_callback(const std_msgs::Float64::ConstPtr& servo_msg)
  {
    RobotData::servo_angle = servo_msg->data;
  }
private:
  ros::Subscriber servo_sub;
};


// Imu subscriber
class I2cImu
{
public:
  explicit I2cImu(ros::NodeHandle node_handle)
  {
    euler_sub = node_handle.subscribe("/localization/imu/euler", 10, &I2cImu::euler_callback, this);
  }

  void euler_callback(const geometry_msgs::Vector3::ConstPtr& euler_msg)
  {
    ImuData::z_angle = euler_msg->z;
  }

private:
  ros::Subscriber euler_sub;
};

class GpioControl
{
  public:
    explicit GpioControl(ros::NodeHandle node_handle)
    {
      hall_sensor_sub = node_handle.subscribe("/localization/encoder/hall_sensor", 10,
                                              &GpioControl::hall_sensor_callback, this);
    }

    void hall_sensor_callback(const std_msgs::Bool::ConstPtr& hall_sensor_msg)
    {
      GPIO::hall_sensor = hall_sensor_msg->data;
    }

  private:
    ros::Subscriber hall_sensor_sub;
};

class Odometry
{
public:
  Odometry();

  void spin();

private:
  std::vector<double> pose_covariance;
  std::vector<double> twist_covariance;

  ros::NodeHandle nh_;

  nav_msgs::Odometry odometry_msg;

  ros::Publisher odometry_pub_;
  ros::Publisher distance_pub_;
  ros::Publisher stopped_pub_;
  ros::Publisher position_pub_;
};

Odometry::Odometry() : nh_()
{
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("wheel_encoder", 10);
  distance_pub_ = nh_.advertise<std_msgs::Float64>("distance", 10);
  stopped_pub_ = nh_.advertise<std_msgs::Bool>("stopped", 10);
  position_pub_ = nh_.advertise<geometry_msgs::Point>("position", 10);

  if (nh_.getParam("pose_covariance", pose_covariance) && pose_covariance.size() == 36)
    for (int i = 0; i < 36; i++)
      odometry_msg.pose.covariance[i] = pose_covariance[i];

  if (nh_.getParam("twist_covariance", twist_covariance) && twist_covariance.size() == 36)
    for (int i = 0; i < 36; i++)
      odometry_msg.twist.covariance[i] = twist_covariance[i];
}

void Odometry::spin()
{
  const double radius = 0.0575;
  const double axes_distance = 0.28;
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  double linear_distance = 0.0;
  double current_dist = 0.0;
  double x = 0.0, y = 0.0;

  I2cImu imu(nh_);
  GpioControl gpio_control(nh_);

  int next_state = !GPIO::hall_sensor;
  nh_.setParam("init_angle", ImuData::z_angle);
  ros::Time last_time = ros::Time::now();
  ros::Time current_time = ros::Time::now();
  while (ros::ok())
  {
    double ds = 0., dx = 0., dy = 0., dt;
    double init_angle;
    bool reset, reversed;
    double x_event, y_event;
    bool enable_set_coordinates;
    ros::spinOnce();
    nh_.getParam("init_angle", init_angle);
    nh_.getParam("reset", reset);
    nh_.getParam("enable_set_coordinates", enable_set_coordinates);
    nh_.getParam("reversed", reversed);
    current_time = ros::Time::now();
    if (enable_set_coordinates)
    {
      nh_.getParam("x_event", x_event);
      nh_.getParam("y_event", y_event);
      x = x_event;
      y = y_event;
      nh_.setParam("enable_set_coordinates", false);
    }
    if (reset)
    {
      current_dist = 0;
      nh_.setParam("reset", false);
    }
    else if (GPIO::hall_sensor == next_state)
    {
      if (!is_gpio_stable(GPIO::hall_sensor, next_state))
        continue;
    }
    else
    {
      continue;
    }
    next_state = !next_state;
    double a = ImuData::z_angle - init_angle;
    ds = M_PI * radius;
    dx = ds * cos(a);
    dy = ds * sin(a);
    dt = (current_time - last_time).toSec();
    last_time = ros::Time::now();
    if (reversed)
    {
      ds = -ds;
      dx = -dx;
      dy = -dy;
    }
    current_dist += ds;
    x += dx;
    y += dy;
    linear_velocity = ds / dt;
    angular_velocity = linear_velocity * sin(RobotData::servo_angle) / axes_distance;
    odometry_msg.header.stamp = current_time;
    odometry_msg.header.frame_id = "odom";
    odometry_msg.child_frame_id = "base_link";
    odometry_msg.twist.twist.linear.x = linear_velocity;
    odometry_msg.twist.twist.linear.y = 0.0;
    odometry_msg.twist.twist.angular.z = angular_velocity;
    odometry_msg.pose.pose.orientation.w = 1.;
    std_msgs::Float64 distance_msg;
    distance_msg.data = current_dist;
    std_msgs::Bool stopped_msg;
    stopped_msg.data = linear_velocity == 0;
    geometry_msgs::Point position_msg;
    position_msg.x = x;
    position_msg.y = y;
    odometry_pub_.publish(odometry_msg);
    distance_pub_.publish(distance_msg);
    stopped_pub_.publish(stopped_msg);
    position_pub_.publish(position_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  Odometry odometry;
  odometry.spin();
  return 0;
}
