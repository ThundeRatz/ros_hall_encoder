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
#include <sensor_msgs/Imu.h>

#include "gpio/gpio.h"

#include <cmath>
#include <iostream>

namespace ImuData
{
  double z_angle, quat_x, quat_y, quat_z, quat_w, angular_velocity;
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
      ROS_INFO("Invalid euler message.");
      return;
    }

    ImuData::z_angle = euler_msg->z;
  }

  void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    if (imu_msg == NULL)
    {
      ROS_INFO("Invalid IMU message.");
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

class Odometry
{
public:
  Odometry();

  void update();
  void spin();

private:
  bool straight;
  bool reset;
  double init_angle;
  double x_event;
  double y_event;
  bool enable_set_coordinates;
  bool reversed;

  ros::NodeHandle nh_;

  nav_msgs::Odometry odometry_msg;
  std_msgs::Float64 distance_msg;
  std_msgs::Bool stopped_msg;
  geometry_msgs::Point position_msg;

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

  nh_.param("straight", straight, true);
  nh_.param("reset", reset, false);
  nh_.param("init_angle", init_angle, 0.0);
  nh_.param("x_event", x_event, 0.0);
  nh_.param("y_event", y_event, 0.0);
  nh_.param("enable_set_coordinates", enable_set_coordinates, false);
  nh_.param("reversed", reversed, false);
}

void Odometry::spin()
{
  GPIO hall_sensor(165);
  I2cImu imu(nh_);

  double radius = 0.0575;
  double linear_velocity = 0.0;
  double linear_distance = 0.0;
  double current_dist = 0.0, last_dist = 0.0;
  double ds = 0.0;
  double x = 0.0, y = 0.0, dx = 0.0, dy = 0.0, dt;
  int next_state = !hall_sensor;
  ros::spinOnce();
  nh_.setParam("init_angle", ImuData::z_angle);
  ros::Time last_time = ros::Time::now();
  ros::Time current_time = ros::Time::now();
  ros::Rate r(20);
  while (ros::ok())
  {
    ros::spinOnce();
    nh_.getParam("init_angle", init_angle);
    nh_.getParam("straight", straight);
    nh_.getParam("reset", reset);
    nh_.getParam("enable_set_coordinates", enable_set_coordinates);
    nh_.getParam("reversed", reversed);
    current_time = ros::Time::now();
    dx = dy = ds = 0;
    if (enable_set_coordinates)
    {
      nh_.getParam("x_event", x_event);
      nh_.getParam("y_event", y_event);
      x = x_event;
      y = y_event;
      nh_.setParam("enable_set_coordinates", false);
    }
    if (linear_velocity == 0)
      stopped_msg.data = true;
    else
      stopped_msg.data = false;
    if (!straight)
    {
      distance_msg.data += 0;
    }
    else if (reset)
    {
      current_dist = last_dist = 0;
      nh_.setParam("reset", false);
    }
    else if (hall_sensor && next_state)
    {
      // If 10 readings or less were 1, consider it an error
      for (int i = 0; i < 10; i++)
      {
        if (!ros::ok())
        {
          ROS_INFO("ROS stopped.");
          return;
        }
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
        if (straight)
        {
          double a = ImuData::z_angle - init_angle;
          ds = M_PI * radius;
          dx = ds * cos(a);
          dy = ds * sin(a);
        }
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
        if (!ros::ok())
        {
          ROS_INFO("ROS stopped.");
          return;
        }
        if (!hall_sensor)
        {
          next_state = 0;
        }
        else
        {
          next_state = 1;
          break;
        }
      }
        // If more than 10 readings were 0, than the state of the sensor
        // is really 0, so the next state must be 1.
        if (!next_state)
        {
          next_state = 1;
          if (straight)
          {
            double a = ImuData::z_angle - init_angle;
            ds = M_PI * radius;
            dx = ds * cos(a);
            dy = ds * sin(a);
          }
        }
        else
        {
          next_state = 0;
        }
    }
    dt = (current_time - last_time).toSec();
    current_dist += ds;
    if (reversed)
    {
      x -= dx;
      y -= dy;
      if (dt > 0.5)
      {
        linear_velocity -= (current_dist - last_dist) / dt;
        last_dist = current_dist;
      }
    }
    else
    {
      x += dx;
      y += dy;
      if (dt > 0.5)
      {
        linear_velocity += (current_dist - last_dist) / dt;
        last_dist = current_dist;
      }
    }
    odometry_msg.header.stamp = current_time;
    odometry_msg.header.frame_id = "odom";
    odometry_msg.child_frame_id = "base_link";
    odometry_msg.twist.twist.linear.x = linear_velocity;
    odometry_msg.twist.twist.angular.z = ImuData::angular_velocity;
    distance_msg.data = current_dist;
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
}
