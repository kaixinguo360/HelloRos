#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

using namespace ros;
using namespace tf;

int main(int argc, char** argv) {

    // 初始化
    init(argc, argv, "my_tf_listener");
    NodeHandle node;

    // 调用服务, 产生第二只乌龟
    service::waitForService("spawn");
    ServiceClient add_turtle_service = 
        node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn req;
    add_turtle_service.call(req);

    // 定义速度发布器
    Publisher turtle_vel_publisher = 
        node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    // TF监听器
    TransformListener listener;

    Rate rate(10.0);
    while (node.ok()) {
        StampedTransform transform;
        Vector3 pos;
        double a, l;

        // 查找两乌龟之间的坐标变换
        try {
            listener.waitForTransform("/turtle1", "/turtle1", Time(0), Duration(3.0));
            listener.lookupTransform("/turtle2", "/turtle1", Time(0), transform);
            pos = transform.getOrigin();
            a = atan2(pos.y(), pos.x());
            l = sqrt(pow(pos.x(), 2) + pow(pos.y(), 2));
        } catch (TransformException &e) {
            ROS_ERROR("%s", e.what());
            Duration(1.0).sleep();
            continue;
        }

        // 计算乌龟2的线速度与角速度并发布
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * a;
        vel_msg.linear.x = 0.5 * l;
        turtle_vel_publisher.publish(vel_msg);
        rate.sleep();
    }

    return 0;
}


