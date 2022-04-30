#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

#ifndef SET_ROUTE_SET_ROUTE_H
#define SET_ROUTE_SET_ROUTE_H

namespace Route
{
    class Set_route
    {
    public:
        explicit Set_route();
        void makePlan();

    
    private:
        ros::NodeHandle n_;
        ros::ServiceClient route_client_;

        void fillPath(nav_msgs::GetPlan::Request &request);
    };
    
}  // namespace Goal_calculate

#endif  // SET_ROUTE_SET_ROUTE_H