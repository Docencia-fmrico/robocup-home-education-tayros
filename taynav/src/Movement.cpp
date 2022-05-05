// Copyright 2022 TayRos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "tf2/convert.h"
#include <ros/ros.h>
#include "transforms.h"
#include "std_msgs/Int32.h"
#include "Movement.h"

namespace follow
{

Movement::Movement()
:// pan_pid_(0.0, 1.0, 0.0, 0.15),
 // tilt_pid_(0.0, 1.0, 0.0, 0.1),
  n("~")
{
  std::string sub_path =  n.param("set_goal_activation_topic", std::string("/tay_ros/set_goal_activation"));
  vel_pub_ = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  act_sub = n.subscribe<std_msgs::Int32>(sub_path, 1, &Movement::callback, this);
  movement_ = STOP;
  increment_ = 4;
}

void
Movement::get_dist_angle_tf()
{
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(1), &error_))
  {
    bf2object_msg_ = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

    tf2::fromMsg(bf2object_msg_, bf2object_);

    dist_ = bf2object_.getOrigin().length();
    angle_ = atan2(bf2object_.getOrigin().y(), bf2object_.getOrigin().x());
    }
    else
    {
      ROS_ERROR("%s", error_.c_str());
      dist_ = 1;
      increment_ = 0;
    }
}

void
Movement::MoveRobot()
{
  if (movement_ == TF)
  {
    ROS_INFO("FOLLOWING: TF");
    get_dist_angle_tf();
  }

  else if (movement_ == STOP)
  {
    ROS_INFO("STOP");
  }

  if (movement_ != STOP)
  {
    //double control_pan = pan_pid_.get_output(angle_);
    //double control_tilt = tilt_pid_.get_output(dist_);

    geometry_msgs::Twist vel_msgs;
    vel_msgs.linear.x = (dist_ - 1.0) / 5;
    vel_msgs.angular.z = angle_ * increment_;
    vel_pub_.publish(vel_msgs);
  }

}
void
Movement::callback(const std_msgs::Int32::ConstPtr& msg)
{
  movement_ = msg->data;
}
}  // namespace follow

int main(int argc, char **argv)
{
  ros::init(argc, argv, "movement");
  follow::Movement movement;

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    movement.MoveRobot();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
