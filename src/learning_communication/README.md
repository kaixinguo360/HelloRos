Hello Ros!
=======

This is my first Ros project.

2020-1-1

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

