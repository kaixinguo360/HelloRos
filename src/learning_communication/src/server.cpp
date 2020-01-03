#include "params.h"
#include "ros/ros.h"
#include "learning_communication/MyService.h"

bool callback(learning_communication::MyService::Request &req,
              learning_communication::MyService::Response &res) {
    res.sum = req.x + req.y;
    ROS_INFO("Request: x=[%ld], y=[%ld]", (long) req.x, (long) req.y);
    ROS_INFO("Response: sum=[%ld]", (long) res.sum);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, SERVER);
    ros::NodeHandle handler;
    ros::ServiceServer server = handler.advertiseService(SERVICE, callback);
    ROS_INFO("My Service server is ready.");
    ros::spin();
}
