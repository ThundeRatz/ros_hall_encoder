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

#include "gpio/gpio.h"

#include <cmath>
#include <iostream>

namespace Parameters
{
  double z_angle;
}

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
    if (euler_msg == NULL)
    {
      ROS_INFO("Invalid euler msg.");
      return;
    }

    Parameters::z_angle = euler_msg->z;
  }
private:
  ros::Subscriber euler_sub;
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

  geometry_msgs::Point odometry_msg;
  std_msgs::Float64 distance_msg;
  std_msgs::Bool stopped_msg;

  ros::Publisher odometry_pub_;
  ros::Publisher distance_pub_;
  ros::Publisher stopped_pub_;
};

Odometry::Odometry() : nh_()
{
  odometry_pub_ = nh_.advertise<geometry_msgs::Point>("data", 10);
  distance_pub_ = nh_.advertise<std_msgs::Float64>("distance", 10);
  stopped_pub_ = nh_.advertise<std_msgs::Bool>("stopped", 10);

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

  double radius = 0.0575, last_dist = 0;
  int next_state = !hall_sensor;
  ros::spinOnce();
  nh_.setParam("init_angle", Parameters::z_angle);
  double timer = ros::Time::now().toSec();
  while (ros::ok())
  {
    nh_.getParam("init_angle", init_angle);
    nh_.getParam("straight", straight);
    nh_.getParam("reset", reset);
    nh_.getParam("enable_set_coordinates", enable_set_coordinates);
    nh_.getParam("reversed", reversed);
    if (enable_set_coordinates)
    {
      nh_.getParam("x_event", x_event);
      nh_.getParam("y_event", y_event);
      odometry_msg.x = x_event;
      odometry_msg.y = y_event;
      nh_.setParam("enable_set_coordinates", false);
    }
    if (ros::Time::now().toSec() - timer >= 1)
      stopped_msg.data = true;
    else
      stopped_msg.data = false;
    if (!straight)
    {
      distance_msg.data += 0;
    }
    else if (reset)
    {
      distance_msg.data = 0;
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
        ros::spinOnce();
        double a = Parameters::z_angle - init_angle;
        if (reversed)
        {
          odometry_msg.x -= M_PI * radius * cos(a);
          odometry_msg.y -= M_PI * radius * sin(a);
          distance_msg.data -= M_PI * radius;
        }
        else if (straight)
        {
          odometry_msg.x += M_PI * radius * cos(a);
          odometry_msg.y += M_PI * radius * sin(a);
          distance_msg.data += M_PI * radius;
        }
        timer = ros::Time::now().toSec();
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
          ros::spinOnce();
          double a = Parameters::z_angle - init_angle;
          if (reversed)
          {
              odometry_msg.x -= M_PI * radius * cos(a);
              odometry_msg.y -= M_PI * radius * sin(a);
              distance_msg.data -= M_PI * radius;
          }
          else if (straight)
          {
              odometry_msg.x += M_PI * radius * cos(a);
              odometry_msg.y += M_PI * radius * sin(a);
              distance_msg.data += M_PI * radius;
          }
          timer = ros::Time::now().toSec();
        }
        else
        {
          next_state = 0;
        }
    }
    odometry_msg.z = 0;
    odometry_pub_.publish(odometry_msg);
    distance_pub_.publish(distance_msg);
    stopped_pub_.publish(stopped_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  Odometry odometry;
  odometry.spin();
}
