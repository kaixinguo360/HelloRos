#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

using namespace tf;

// 乌龟name
std::string turtle_name;

// 收到乌龟pose订阅消息时的回调函数
// 参数msg: Pose结构体， 代表乌龟位置
//  Pose_()
//    : x(0.0)
//    , y(0.0)
//    , theta(0.0)
//    , linear_velocity(0.0)
//    , angular_velocity(0.0)  {
//    }
void poseCallback(const turtlesim::PoseConstPtr& msg) {

    // 定义TF广播器
    static TransformBroadcaster br;

    // 根据当前乌龟位置， 设置相对于世界坐标系的坐标变换
    Transform transform;
    transform.setOrigin(Vector3(msg->x, msg->y, 0.0)); // 设置位置(X,Y,Z)
    Quaternion q;
    q.setRPY(0, 0, msg->theta); // 设置旋转角度(X,Y,Z)
    transform.setRotation(q);

    // 发送坐标变换
    // StampedTransform(
    //      const tf::Transform& input,
    //      const ros::Time& timestamp,
    //      const std::string & frame_id,
    //      const std::string & child_frame_id):
    br.sendTransform(StampedTransform(transform, ros::Time::now(),
                "world", turtle_name));
}


int main(int argc, char** argv) {

    // 初始化
    ros::init(argc, argv, "my_tf_broadcaster");

    // 读取乌龟name
    if (argc != 2) {
        ROS_ERROR("Need turtle name as argument!");
        return -1;
    }
    turtle_name = argv[1];

    // 订阅乌龟pose信息
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name + "/pose",
            10, &poseCallback);

    // 进入循环
    ros::spin();

    return 0;
}


