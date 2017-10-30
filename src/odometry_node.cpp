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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include "gpio/gpio.h"

#include <cmath>
#include <iostream>

namespace ImuData
{
  double z_angle;
  double angular_velocity;
  double quat_x;
  double quat_y;
  double quat_z;
  double quat_w;
}

// Imu subscriber
class I2cImu
{
public:
  explicit I2cImu(ros::NodeHandle node_handle)
  {
    euler_sub = node_handle.subscribe("/localization/imu/euler", 10, &I2cImu::euler_callback, this);
    imu_sub = node_handle.subscribe("/localization/imu/data", 10, &I2cImu::imu_callback, this);
  }

  void euler_callback(const geometry_msgs::Vector3::ConstPtr& euler_msg)
  {
    if (euler_msg == NULL)
    {
      ROS_INFO("Invalid euler msg.");
      return;
    }

    ImuData::z_angle = euler_msg->z;
  }

  void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    if (imu_msg == NULL)
    {
      ROS_INFO("Invalid IMU msg.");
      return;
    }
    ImuData::angular_velocity = imu_msg->angular_velocity.z;
    ImuData::quat_x = imu_msg->orientation.x;
    ImuData::quat_y = imu_msg->orientation.y;
    ImuData::quat_z = imu_msg->orientation.z;
    ImuData::quat_w = imu_msg->orientation.w;
  }
private:
  ros::Subscriber euler_sub;
  ros::Subscriber imu_sub;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  GPIO hall_sensor(164);

  ros::NodeHandle nh_;
  ros::Publisher odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  nav_msgs::Odometry odometry_msg;
  I2cImu imu(nh_);
  odometry_msg.pose.pose.position.z = 0.0;
  odometry_msg.twist.twist.linear.z = 0.0;
  odometry_msg.twist.twist.angular.x = 0.0;
  odometry_msg.twist.twist.angular.y = 0.0;
  
  double x = 0.0;
  double y = 0.0;
  
  double vx = 0.0;
  double vy = 0.0;

  double dt;
  double dx = 0;
  double dy = 0;

  double radius = 0.055, last_dist = 0;
  int next_state = !hall_sensor;
  ros::spinOnce();
  double angle_ini = ImuData::z_angle;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(50.0);
  while (nh_::ok())
  {
    dx = 0;
    dy = 0;
    ros::spinOnce();
    nh_.getParam("straight", straight);
    nh_.getParam("reset", reset);
    if (vx == 0 && vy == 0)
      stopped_msg.data = true;
    else
      stopped_msg.data = false;
    if (!straight)
    {
      distance_msg.data = 0;
    }
    else if (reset)
    {
      distance_msg.data = -1;
      nh_.setParam("reset", false);
    }
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    else if (hall_sensor && next_state)
    {
      // If 10 readings or less were 1, consider it an error
      for (int i = 0; i < 10; i++)
      {
        if (hall_sensor)
        {
          next_state = 1;
        }
        else
        {
          next_state = 0;
          break;
        }
      }
      // If more than 10 readings were 1, than the state of the sensor
      // is really 1, so the next state must be 0.
      if (next_state)
      {
        next_state = 0;
        double a = ImuData::z_angle - angle_ini;
        dx = M_PI * radius * cos(a);
        dy = M_PI * radius * sin(a);
        distance_msg.data = M_PI * radius;
      }
      else
      {
        next_state = 1;
      }
    }

    else if (!(hall_sensor) && !(next_state))
    {
      // If 10 readings or less were 0, consider it an error
      for (int i = 0; i < 10; i++)
      {
        if (!hall_sensor)
        {
          next_state = 0;
        }
        else
        {
          next_state = 1;
          break;
        }
        // If more than 10 readings were 0, than the state of the sensor
        // is really 0, so the next state must be 1.
        if (!next_state)
        {
          next_state = 1;
          double a = ImuData::z_angle - angle_ini;
          dx = M_PI * radius * cos(a);
          dy = M_PI * radius * sin(a);
          distance_msg.data = M_PI * radius;
        }
        else
        {
          next_state = 0;
        }
      }
    }
    x += dx;
    y += dy;
    if (dt > 0.5)
    {
      vx = dx / dt;
      vy = dy / dt;
      last_time = current_time;
    }
    odometry_msg.pose.pose.position.x = x;
    odometry_msg.pose.pose.position.y = y;
    odometry_msg.twist.twist.linear.x = vx;
    odometry_msg.twist.twist.linear.y = vy;
    odometry_msg.pose.pose.orientation.x = ImuData::quat_x;
    odometry_msg.pose.pose.orientation.y = ImuData::quat_y;
    odometry_msg.pose.pose.orientation.z = ImuData::quat_z;
    odometry_msg.pose.pose.orientation.w = ImuData::quat_w;
    odometry_pub_.publish(odometry_msg);
    distance_pub_.publish(distance_msg);
    stopped_pub_.publish(stopped_msg);
  }
}
