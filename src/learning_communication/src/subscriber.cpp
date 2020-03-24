#include "const.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heared: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, SUBSCRIBER);
    ros::NodeHandle handler;
    ros::Subscriber sub = handler.subscribe(TOPIC, 1000, callback);
    ros::spin();
}
