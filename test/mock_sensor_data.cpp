/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 ThundeRatz

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
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>

#include <cmath>
#include <vector>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mock_sensor_data_publisher");
  ros::NodeHandle node_handle;
  ros::Publisher mock_hall_sensor_pub;
  ros::Publisher mock_imu_euler_pub;
  std_msgs::Bool hall_sensor_msg;
  geometry_msgs::Vector3 imu_euler_msg;
  mock_hall_sensor_pub = node_handle.advertise<std_msgs::Bool>("/localization/encoder/hall_sensor", 10);
  mock_imu_euler_pub = node_handle.advertise<geometry_msgs::Vector3>("/localization/imu/euler", 10);
  bool turn_left;
  double time_elapsed = 0;
  double angle = 0;
  hall_sensor_msg.data = false;
  ros::Time begin = ros::Time::now();
  ros::Rate r(80);
  node_handle.param("turn_left", turn_left, true);
  while (ros::ok())
  {
    time_elapsed = (ros::Time::now() - begin).toSec();
    if (time_elapsed > 0.2)
    {
      if (turn_left)
      {
        angle += 0.1;
      }
      else
      {
        angle -= 0.1;
      }
      if (angle < 0)
      {
        angle += 2 * M_PI;
      }
      if (angle >= 2 * M_PI)
      {
        angle -=  2 * M_PI;
      }
      imu_euler_msg.x = 0;
      imu_euler_msg.y = 0;
      imu_euler_msg.z = angle;
      hall_sensor_msg.data = !hall_sensor_msg.data;
      begin = ros::Time::now();
    }
    mock_hall_sensor_pub.publish(hall_sensor_msg);
    mock_imu_euler_pub.publish(imu_euler_msg);
    r.sleep();
  }
  return 0;
}
