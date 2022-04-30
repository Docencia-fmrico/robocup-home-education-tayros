#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include "Set_route/set_route.h"

namespace Route
{

Set_route::Set_route()
{   std::string service_name = "/move_base/make_plan";
    route_client_ = n_.serviceClient<nav_msgs::GetPlan>(service_name);
}

void 
Set_route::fillPath(nav_msgs::GetPlan::Request &request)
{
    request.start.header.frame_id ="map";
    request.start.pose.position.x = 12.378; 
    request.start.pose.position.y = 28.638 ; 
    request.start.pose.orientation.w = 1.0 ; 
    request.goal.header.frame_id = "map" ; 
    request.goal.pose.position.x = 5.00; 
    request.goal.pose.position.y = 3.00; 
    request.goal.pose.orientation.w = 1.0 ; 
    request.tolerance = 0.5 ; 
}

void
Set_route::makePlan()
{
    nav_msgs::GetPlan goal;
    fillPath(goal.request);

    if (route_client_.call(goal))
    {
        ROS_INFO("Service sucessfull?");
    }
    else
    {
        ROS_ERROR("Try another time bruh");
    }
}

}