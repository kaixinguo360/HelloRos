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

