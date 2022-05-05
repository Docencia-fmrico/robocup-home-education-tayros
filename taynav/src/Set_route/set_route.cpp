#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <string>
#include "Set_route/set_route.h"
#include <geometry_msgs/PolygonStamped.h>

namespace Route
{

Set_route::Set_route() : n_("~")
{   
    std::string human_pos_topic =  n_.param("tf_to_pos_topic", std::string("/tayros/pose_of_human"));
    std::string service_name = "/move_base/make_plan";
    std::string pos_topic_pub =  n_.param("target_pos_topic", std::string("/tayros/pose_to_go"));

    route_client_ = n_.serviceClient<nav_msgs::GetPlan>(service_name);
    sub_target_pos =  n_.subscribe<move_base_msgs::MoveBaseGoal>(human_pos_topic, 1, &Set_route::human_pos_callback, this);
    sub_robot_pos_ = n_.subscribe<geometry_msgs::PolygonStamped>("/move_base/global_costmap/footprint", 1, &Set_route::current_pos_callback, this);
    calculated_pos_pub_ = n_.advertise<move_base_msgs::MoveBaseGoal>(pos_topic_pub, 1);

    currentX_ = 0;
    currentY_= 0;
    msg_recived_ = false;
}

void 
Set_route::fillPath(nav_msgs::GetPlan::Request &request)
{
    request.start.header.frame_id ="map";
    request.start.pose.position.x = currentX_; 
    request.start.pose.position.y = currentY_; 
    request.start.pose.orientation.w = DEFAULT_ORIENTATION_W; 

    request.goal.header.frame_id = "map"; 
    request.goal.pose.position.x = targetX_; 
    request.goal.pose.position.y = targetY_; 
    request.goal.pose.orientation.w = DEFAULT_ORIENTATION_W; 
    request.tolerance = tolerance; 
}

void
Set_route::makePlan()
{
    nav_msgs::GetPlan new_goal;
    move_base_msgs::MoveBaseGoal goal;
    fillPath(new_goal.request);

    if (route_client_.call(new_goal))
    {
        long index = size(new_goal.response.plan.poses);
        if (index != 0)
        {
            int array_pos = index-1-(index-1)*FACTOR;

            ROS_INFO("Longitud array = %d, Array_pos = %d\n", index, array_pos);
            goal.target_pose.pose.position.x = new_goal.response.plan.poses[array_pos].pose.position.x;
            goal.target_pose.pose.position.y = new_goal.response.plan.poses[array_pos].pose.position.y;
            goal.target_pose.pose.orientation.x = new_goal.response.plan.poses[array_pos].pose.orientation.x;
            goal.target_pose.pose.orientation.y = new_goal.response.plan.poses[array_pos].pose.orientation.y;
            goal.target_pose.pose.orientation.z = new_goal.response.plan.poses[array_pos].pose.orientation.z;
            goal.target_pose.pose.orientation.w = new_goal.response.plan.poses[array_pos].pose.orientation.w;

            if (msg_recived_)
            {
                calculated_pos_pub_.publish(goal);
                msg_recived_ = false;
            }
        }
    }
    else
    {
        ROS_ERROR("Service failed");
    }
}

void
Set_route::human_pos_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& target)
{
    targetX_ = target->target_pose.pose.position.x;
    targetY_ = target->target_pose.pose.position.y;
    orientationX = target->target_pose.pose.orientation.x;
    orientationY = target->target_pose.pose.orientation.y;
    orientationZ = target->target_pose.pose.orientation.z;
    orientationW = target->target_pose.pose.orientation.w;
    msg_recived_ = true;
}

void
Set_route::current_pos_callback(const geometry_msgs::PolygonStamped::ConstPtr& position)
{
    currentX_ = position->polygon.points[0].x;
    currentY_ = position->polygon.points[0].y;
    ROS_INFO("Pos x = %f, Pos Y = %f", currentX_, currentY_);
}

}