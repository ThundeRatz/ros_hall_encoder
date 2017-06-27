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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include "gpio/gpio.h"

#include <cmath>
#include <iostream>

double z_angle;

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

    z_angle = euler_msg->z;
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
  ros::NodeHandle nh_;
  geometry_msgs::Point odometry_msg;

  ros::Publisher odometry_pub_;

  ros::Time last_update_;
};

Odometry::Odometry() : nh_()
{
  odometry_pub_ = nh_.advertise<geometry_msgs::Point>("data", 10);
}

void Odometry::update()
{
  GPIO hall_sensor(164);
  I2cImu imu(nh_);

  double radius = 0.1, last_dist = 0;
  int next_state = !hall_sensor;

  ros::spinOnce();
  double angle_ini = z_angle;

  while (ros::ok())
  {
    if (hall_sensor && next_state)
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
          i = 10;
          next_state = 0;
        }
      }
      // If more than 10 readings were 1, than the state of the sensor
      // is really 1, so the next state must be 0.
      if (next_state)
      {
        next_state = 0;
        // if(straight)
        // {
        // dist += M_PI * radius;
        ros::spinOnce();
        double a = z_angle - angle_ini;
        odometry_msg.x += M_PI * radius * cos(a);
        odometry_msg.y += M_PI * radius * sin(a);
        // }
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
          i = 10;
          next_state = 1;
        }
        // If more than 10 readings were 0, than the state of the sensor
        // is really 0, so the next state must be 1.
        if (!next_state)
        {
          next_state = 1;
          // if(straight)
          // {
          // dist += M_PI * radius;
          ros::spinOnce();
          double a = z_angle - angle_ini;
          odometry_msg.x += M_PI * radius * cos(a);
          odometry_msg.y += M_PI * radius * sin(a);
          // }
        }
        else
        {
          next_state = 0;
        }
      }
    }
    odometry_msg.z = 0;
    odometry_pub_.publish(odometry_msg);
  }
}

void Odometry::spin()
{
  ros::Rate r(1.0);
  while (ros::ok())
  {
    update();
    r.sleep();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");

  Odometry odometry;
  odometry.spin();
}
