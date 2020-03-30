#!/usr/bin/python2
# coding=utf-8
import moveit_commander
import rospy
import sys
from PyKDL import *

from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from moveit_msgs.msg import RobotTrajectory


class Arm:
    cmd = ''
    end_link = ''
    wait = True

    def __init__(self, node, arm='arm', ref_frame='base_link'):
        # 等待MoveIt服务上线
        rospy.wait_for_service('/move_group/load_map')

        # 初始化环境
        moveit_commander.roscpp_initialize(sys.argv)  # 初始化MoveIt
        rospy.init_node(node)  # 初始化Ros节点

        # 初始化属性
        self.cmd = moveit_commander.MoveGroupCommander(arm)  # 获取MoveGroupCommander
        self.end_link = self.cmd.get_end_effector_link()  # 获取终端link的名称

        # 设置参数
        self.cmd.set_pose_reference_frame(ref_frame)  # 目标位置参考坐标系
        self.cmd.allow_replanning(True)  # 允许重新规划 (5次)
        self.cmd.set_goal_position_tolerance(0.01)  # 位移允许误差
        self.cmd.set_goal_orientation_tolerance(0.05)  # 旋转允许误差

    def destroy(self):
        # 关闭节点
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # ---- utils ---- #

    def go(self):
        self.cmd.go(wait=self.wait)

    def goto_named_target(self, name):
        self.cmd.set_named_target(name)
        self.go()

    def get_transform(self):
        pose = self.cmd.get_current_pose(self.end_link).pose
        return pose_to_frame(pose)

    # ---- transform ---- #

    def set_to_transform(self, transform):
        pose = None
        if isinstance(transform, Frame):
            pose = frame_to_pose(transform)
        elif isinstance(transform, Pose):
            pose = transform
        self.cmd.set_start_state_to_current_state()
        self.cmd.set_pose_target(pose, self.end_link)
        plan = self.cmd.plan()
        self.cmd.execute(plan)
    def transform(self, transform):
        self.set_to_transform(self.get_transform() * transform)

    def set_to_translate(self, x=0, y=0, z=0):
        self.set_to_transform(Frame(Vector(x, y, z)))
    def translate(self, x=0, y=0, z=0):
        self.transform(Frame(Vector(x, y, z)))

    def set_to_rotate(self, axis_x=0, axis_y=0, axis_z=0):
        self.set_to_transform(Frame(Rotation.RPY(axis_x, axis_y, axis_z)))
    def rotate(self, axis_x=0, axis_y=0, axis_z=0):
        self.transform(Frame(Rotation.RPY(axis_x, axis_y, axis_z)))


def frame_to_pose(frame):
    assert isinstance(frame, Frame)
    pose = Pose()

    assert isinstance(frame.p, Vector)
    vector = frame.p
    pose.position.x = vector.x()
    pose.position.y = vector.y()
    pose.position.z = vector.z()

    assert isinstance(frame.M, Rotation)
    quaternion = frame.M.GetQuaternion()
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose


def pose_to_frame(pose):
    if isinstance(pose, PoseStamped):
        pose = pose.pose
    assert isinstance(pose, Pose)

    pos = Vector(
        pose.position.x,
        pose.position.y,
        pose.position.z
    )

    rot = Rotation.Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )

    frame = Frame(rot, pos)

    return frame
