#include <string>
#include "Set_goal_node.h"
#include "ros/ros.h"
#include <unistd.h>
#include "std_msgs/Int32.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "transforms.h"

enum
{
    STOP = 0,
    GO = 1,
};


namespace Goal_calculate
{

SetGoal::SetGoal() : nh_("~")
{
  posX_ = 0.0;
  posY_ = 0.0;
  posZ_ = 0.0;

  roll_ = 0.0;
  pitch_ = 0.0;
  yaw_ = 0.0;

  transform_made_ = false;

  std::string mov_publisher =  nh_.param("tf_to_pos_topic", std::string("/tayros/tf_to_poses"));
  std::string activation_sub =  nh_.param("set_goal_activation_topic", std::string("tay_ros/set_goal_activation"));
  pos_pub_ = nh_.advertise<move_base_msgs::MoveBaseGoal>(mov_publisher, 1);
  sub_activation_ =  nh_.subscribe<std_msgs::Int32>(activation_sub, 1, &SetGoal::callback, this);
}

void
SetGoal::get_dist_angle_tf()
{
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  if (buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(1), &error_))
  {
    map2odom_msg_ = buffer.lookupTransform("map", "odom", ros::Time(0));

    tf2::fromMsg(map2odom_msg_, map2odom_);

    tf2::Matrix3x3(map2odom_.getRotation()).getRPY(roll_, pitch_, yaw_);
    
    transform_made_ = true;

    ROS_INFO("TRANSFORMATION DONE");
    std::cout << "X: " << posX_ << " Y: " << posY_ << " Z: " << posZ_ << std::endl;
  }
  else
  {
    ROS_ERROR("%s", error_.c_str());
    transform_made_ = false;
  }

  if (buffer.canTransform("odom", "base_footprint", ros::Time(0), ros::Duration(1), &error_))
  {
    odom2bf_msg_ = buffer.lookupTransform("odom", "base_footprint", ros::Time(0));

    tf2::fromMsg(odom2bf_msg_, odom2bf_);

    tf2::Matrix3x3(odom2bf_.getRotation()).getRPY(roll_, pitch_, yaw_);
    
    transform_made_ = true;

    ROS_INFO("TRANSFORMATION DONE");
    std::cout << "X: " << posX_ << " Y: " << posY_ << " Z: " << posZ_ << std::endl;
  }
  else
  {
    ROS_ERROR("%s", error_.c_str());
    transform_made_ = false;
  }

  if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(1), &error_))
  {
    bf2person_msg_ = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

    tf2::fromMsg(bf2person_msg_, bf2person_);

    tf2::Matrix3x3(bf2person_.getRotation()).getRPY(roll_, pitch_, yaw_);
    
    transform_made_ = true;

    ROS_INFO("TRANSFORMATION DONE");
    std::cout << "X: " << posX_ << " Y: " << posY_ << " Z: " << posZ_ << std::endl;
  }
  else
  {
    ROS_ERROR("%s", error_.c_str());
    transform_made_ = false;
  }

  map2person_ = map2odom_ * odom2bf_ * bf2person_;

  posX_ = map2person_.getOrigin().x()-1;
  posY_ = map2person_.getOrigin().y();
  posZ_ = map2person_.getOrigin().z();

  tf2::Matrix3x3(map2person_.getRotation()).getRPY(roll_, pitch_, yaw_);
}

void
SetGoal::calculate_goal()
{
  activation_ = GO;
  if (activation_ == GO)
  {
   // get_dist_angle_tf();
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = posX_;
    goal.target_pose.pose.position.y = posY_;
    goal.target_pose.pose.position.z = posZ_;
    goal.target_pose.pose.orientation.x = roll_;
    goal.target_pose.pose.orientation.y = yaw_;
    goal.target_pose.pose.orientation.z = pitch_;
    goal.target_pose.pose.orientation.w = 1.0;

    pos_pub_.publish(goal);
    ROS_INFO("Ejecutando...");
  }

}

void
SetGoal::callback(const std_msgs::Int32::ConstPtr& msg)
{
  activation_ = msg->data;
}

}  // namespace Goal_calculate

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_in_human");
    Goal_calculate::SetGoal goalPublisher;
    ros::Rate loop_rate(20);
    while (ros::ok())
    {

        goalPublisher.calculate_goal();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}