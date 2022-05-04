#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PolygonStamped.h>
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
        ros::Subscriber sub_target_pos;
        ros::Publisher calculated_pos_pub_;
        ros::Subscriber sub_robot_pos_;

        void human_pos_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& target);
        void current_pos_callback(const geometry_msgs::PolygonStamped::ConstPtr& position);
        void fillPath(nav_msgs::GetPlan::Request &request);

        float FACTOR = 0.1;
        float tolerance = 1;

        float targetX_;
        float targetY_;
        float orientationX;
        float orientationY;
        float orientationZ;
        float orientationW;
        float DEFAULT_ORIENTATION_W = 1;

        bool first_msg_recived_;
        float currentX_;
        float currentY_;
    };
    
}  // namespace Goal_calculate

#endif  // SET_ROUTE_SET_ROUTE_H