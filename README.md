Hello Ros!
=======

This is my first Ros project.

Create Workspace
-------

```bash
# Create workspace
mkdir ./src
cd ./src
catkin_init_workspace

# Build
cd ..
catkin_make
source ./devel/setup.bash
```

Create Package
-------

```bash
# Create package
cd ./src
catkin_create_pkg learning_communicating std_msgs rospy roscpp
cd ..
catkin_make
source ./devel/setup.bash
```

launch启动文件
-------

```xml
<launch>

  <node
    pkg="package_name"
    type="executable_file_name"
    name="node_name"

    output="screen"
    respawn="true"
    required="true"
    ns="namespace"
    args="arguments"
  />

  <param name="param_name" value="param_value"/>
  <rosparam file="$(arg params_yaml_file)" command="load" ns="namespace"/>

  <arg name="arg_name" default="arg_value"/>

  <remap from="/from/node/name" to="/new/node/name"/>

  <include file="$(dirname)/other.launch"/>

</launch>
```

一般放置在`<package>/launch`文件夹下, 并且需要向`<package>/CMakeLists.txt`中添加几句:
```CMake
install(
    FILES
    DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

TF坐标系
-------

```bash
# 查看所有坐标系/指定坐标系之间发布状态
rosrun tf tf_monitor <framea> <frameb>

# 查看坐标系间关系变换
rosrun tf tf_echo <framea> <frameb>

# 显示整棵TF树信息， 生成PDF
rosrun tf view_frames
evince frame.pdf

# 发布静态坐标系
rosrun tf static_transform_publisher ...
```

```cpp
using namespace ros;
using namespace tf;

// TF广播器
TransformBroadcaster br;
br.sendTransform(StampedTransform(transform, Time::now(), "parent", "child"));


// TF监听器
TransformListener listener;
listener.waitForTransform("parent", "child", Time(0), Duration(3.0));
listener.lookupTransform("parent", "child", Time(0), transform);

// 变换
Transform transform;
transform.setOrigin(Vector3(x, y, z)); // 设置位置(X,Y,Z)
Quaternion q;
q.setRPY(roll, pitch, yaw); // 设置旋转角度(X,Y,Z)
transform.setRotation(q);
```

实用工具
-------

```bash
# Qt工具箱
rqt_console     # 日志输出工具
rqt_graph       # 计算图可视化工具
rqt_plot        # 数据绘图工具
rqt_reconfigure # 参数动态配置工具

# rviz三维可视化平台
rosrun rviz rviz

# Gazebo仿真环境
rosrun gazebo_ros gazebo

# rosbag数据记录与回放
rosbag record -a
rosbag info <bagfile>
rosbag play <bagfile>
```

