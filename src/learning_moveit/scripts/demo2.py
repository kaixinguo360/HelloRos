#!/usr/bin/env python


# Author: Francis

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=1)


def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4):
    box_name = "box"

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():

        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        is_known = box_name in scene.get_known_object_names()

        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        rospy.sleep(0.1)
        seconds = rospy.get_time()

    return False


def creat_box(scene, group, pose=[]):
    rospy.sleep(2.0)

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.orientation.w = pose[0]
    # box_pose.pose.orientation.x = 0.0
    # box_pose.pose.orientation.y = 0.0
    # box_pose.pose.orientation.z = 0.0
    box_pose.pose.position.x = pose[1]
    box_pose.pose.position.y = pose[2]
    box_pose.pose.position.z = pose[3]
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(pose[4], pose[5], pose[6]))
    wait_for_state_update(box_is_known=True, timeout=5)

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""


def go_to_pose(robot, group, pose=[]):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]

    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]
    group.set_pose_target(pose_goal)

    group.go(wait=True)
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""


creat_box(scene, group, [1.0, 0.2, 0.2, 0.25, 0.1, 0.1, 0.5])

go_to_pose(robot, group, [0.30603, 0.017247, 0.64808, 0.59731, 0.52117, -0.4175, -0.44418])
# rospy.sleep(10.0)
go_to_pose(robot, group, [0.090837, 0.42689, 0.19629, 0.92343, 0.38265, -0.026938, -0.011112])

# result = group.go(wait=True)

group.stop()

group.clear_pose_targets()

rospy.sleep(10)

# moveit_commander.roscpp_shutdown()
