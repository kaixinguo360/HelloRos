#include "const.h"
#include "ros/ros.h"
#include "learning_communication/MyService.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, CLIENT);

    if (argc != 3) {
        ROS_INFO("My Service Client\ndescription: add two ints\nusage: %s <intX> <intY>", argv[0]);
        return 1;
    }

    ros::NodeHandle handler;
    ros::ServiceClient client = handler.serviceClient<learning_communication::MyService>(SERVICE);

    learning_communication::MyService srv;
    srv.request.x = atol(argv[1]);
    srv.request.y = atol(argv[2]);

    if (client.call(srv)) {
        ROS_INFO("Sum: %ld", (long) srv.response.sum);
    } else {
        ROS_INFO("Failed to call service `%s`", SERVICE);
        return 1;
    }

    return 0;
}
