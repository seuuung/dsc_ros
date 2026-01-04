#include "ros/ros.h"
#include "std_msgs/Int16.h"

void VelCmdCallback(const std_msgs::Int16::ConstPtr& msg)
{
ROS_INFO("vel_cmd_check: [%d]", msg->data);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("vel_cmd", 1000, VelCmdCallback);
    ros::spin();
    return 0;
}