#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include "Set_route/set_route.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "set_route_service");
    Route::Set_route route;
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        route.makePlan();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
