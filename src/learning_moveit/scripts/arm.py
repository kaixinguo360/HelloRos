#!/usr/bin/python2
# coding=utf-8
import moveit_commander
import rospy

from PyKDL import Frame, Vector, Rotation
from geometry_msgs.msg import Pose

from utils import frame_to_pose, pose_to_frame


class Arm:
    cmd = None
    end_link = ''
    wait = True
    cartesian = False

    def __init__(self, arm_name='arm', ref_frame='base_link'):

        # 初始化属性
        self.cmd = moveit_commander.MoveGroupCommander(arm_name)  # 获取MoveGroupCommander
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

    def get_pose(self):
        return self.cmd.get_current_pose(self.end_link).pose

    def get_transform(self):
        return pose_to_frame(self.get_pose())

    def get_translation(self):
        return self.get_transform().p

    def get_rotation(self):
        return self.get_transform().M

    # ---- absolute transform ---- #

    def to_target(self, name):
        self.cmd.set_named_target(name)
        self.cmd.go(wait=self.wait)

    def to_transform(self, xyz, rpy=None, cartesian=None):
        pose = None

        if isinstance(xyz, Frame):
            pose = frame_to_pose(xyz)
        elif isinstance(xyz, Pose):
            pose = xyz
        elif isinstance(xyz, tuple) and isinstance(rpy, tuple):
            pos = Vector(xyz[0], xyz[1], xyz[2])
            rot = Rotation.RPY(rpy[0], rpy[1], rpy[2])
            pose = frame_to_pose(Frame(rot, pos))

        if cartesian is None:
            cartesian = self.cartesian

        self.cmd.set_start_state_to_current_state()
        if cartesian:
            way_points = [self.get_pose(), pose]
            fraction = 0.0      # 路径规划覆盖率
            max_tries = 100     # 最大尝试次数
            attempts = 0        # 已经尝试次数
            while fraction < 1.0 and attempts < max_tries:
                (plan, fraction) = self.cmd.compute_cartesian_path(
                    way_points,     # 路点列表
                    0.001,          # 步进值
                    0.0,            # 跳跃阈值
                    True            # 避障规划
                )
                attempts += 1
                # if attempts % 10 == 0:
                #     rospy.loginfo("Still trying after " +
                #                   str(attempts) + " attempts... (" +
                #                   str(fraction) + ")")
            if fraction == 1.0:
                # rospy.loginfo("Path computed successfully. Moving the arm.")
                self.cmd.execute(plan)
                # rospy.loginfo("Path execution")
            else:
                rospy.loginfo("Path planning failed with only " +
                              str(fraction) + " success after " +
                              str(max_tries) + " attempts. Give up Cartesian mode.")
                cartesian = False

        if not cartesian:
            self.cmd.set_pose_target(pose, self.end_link)
            plan = self.cmd.plan()
            self.cmd.execute(plan)

    def to_translate(self, xyz, cartesian=None):
        cur = self.get_transform()
        cur.p = Vector(xyz[0], xyz[1], xyz[2])
        self.to_transform(cur, cartesian=cartesian)

    def to_rotate(self, rpy, cartesian=None):
        cur = self.get_transform()
        cur.M = Rotation.RPY(rpy[0], rpy[1], rpy[2])
        self.to_transform(cur, cartesian=cartesian)

    # ---- local transform ---- #

    def local_transform(self, transform, cartesian=None):
        self.to_transform(self.get_transform() * transform, cartesian=cartesian)
    def local_translate(self, xyz, cartesian=None):
        self.local_transform(Frame(Vector(xyz[0], xyz[1], xyz[2])), cartesian=cartesian)
    def local_rotate(self, rpy, cartesian=None):
        self.local_transform(Frame(Rotation.RPY(rpy[0], rpy[1], rpy[2])), cartesian=cartesian)

    # ---- global transform ---- #

    def global_transform(self, transform, cartesian=None):
        self.to_transform(transform * self.get_transform(), cartesian=cartesian)
    def global_translate(self, xyz, cartesian=None):
        self.global_transform(Frame(Vector(xyz[0], xyz[1], xyz[2])), cartesian=cartesian)
    def global_rotate(self, rpy, cartesian=None):
        self.global_transform(Frame(Rotation.RPY(rpy[0], rpy[1], rpy[2])), cartesian=cartesian)
