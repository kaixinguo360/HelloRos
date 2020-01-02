#include <sstream>
#include "params.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, PUBLISHER);
    ros::NodeHandle handler;
    ros::Publisher publisher = handler.advertise<std_msgs::String>(TOPIC, 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
        std::stringstream ss;
        ss << "Hello,world! (" << count << ")";
        
        std_msgs::String msg;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}
