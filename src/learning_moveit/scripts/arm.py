#!/usr/bin/python2
# coding=utf-8
from copy import deepcopy

import moveit_commander
import rospy

from PyKDL import Frame, Vector, Rotation
from genpy import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes, PlaceLocation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from utils import frame_to_pose, pose_to_frame, build_frame


class Arm:
    cmd = None  # type: moveit_commander.MoveGroupCommander
    wait = True  # type: bool

    ref_frame = ''  # type: str
    end_link = ''  # type: str

    cartesian = False  # type: bool
    way_points = []  # type: list
    max_tries = 10  # type: int
    auto_commit = True  # type: bool

    open_posture = None
    close_posture = None

    hand_offset = build_frame((0, 0, -0.3), (0, 0, 0))  # type: Frame

    def __init__(self, arm_name='arm', ref_frame='base_link'):

        # 初始化属性
        self.cmd = moveit_commander.MoveGroupCommander(arm_name)  # 获取MoveGroupCommander
        self.end_link = self.cmd.get_end_effector_link()  # 获取终端link的名称
        self.ref_frame = ref_frame  # 设置参考link的名称

        # 设置参数
        self.cmd.set_pose_reference_frame(ref_frame)  # 目标位置参考坐标系
        self.cmd.allow_replanning(True)  # 允许重新规划 (5次)
        self.cmd.set_goal_position_tolerance(0.01)  # 位移允许误差
        self.cmd.set_goal_orientation_tolerance(0.05)  # 旋转允许误差

        self.open_posture = self.JointTrajectory(0.09, 0.5)
        self.close_posture = self.JointTrajectory(0.06, 0.5)

    def destroy(self):
        # 关闭节点
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # ---- getter ---- #

    def get_pose(self):
        return self.cmd.get_current_pose(self.end_link).pose

    def get_transform(self):
        return pose_to_frame(self.get_pose())

    def get_translation(self):
        return self.get_transform().p

    def get_rotation(self):
        return self.get_transform().M

    # ---- pick & place ---- #

    def pick(
            self,
            target_name,
            target_transform,
            pre_grasp_approach,
            post_grasp_retreat,
            post_place_retreat=None,
            open_grasp_posture=0.09,
            close_grasp_posture=0.06,
            grasp_timeout=0.5,
            max_attempts=10
    ):
        g = Grasp()
        g.id = target_name + '_pick_grasp'

        # 抓取时的姿态
        g.grasp_pose.header.frame_id = self.ref_frame
        g.grasp_pose.pose = frame_to_pose(target_transform)

        # 夹持器张开和闭合的姿态
        g.pre_grasp_posture = self.open_posture
        g.grasp_posture = self.close_posture

        # 接近与撤离目标的参数
        g.pre_grasp_approach = pre_grasp_approach
        g.post_grasp_retreat = post_grasp_retreat
        g.post_place_retreat = post_place_retreat

        # 其他参数
        g.grasp_quality = 0.5
        g.max_contact_force = 100
        g.allowed_touch_objects = [target_name]

        # self.cmd.set_support_surface_name('ground')

        result = False
        attempts = 0
        while result != MoveItErrorCodes.SUCCESS and attempts < max_attempts:
            result = self.cmd.pick(target_name, g)
            rospy.sleep(0.01)
            attempts += 1

    def place(
            self,
            target_name,
            target_transform,
            pre_place_approach,
            post_place_retreat,
            open_place_posture=0.09,
            place_timeout=0.5,
            max_attempts=10
    ):
        p = PlaceLocation()

        p.id = target_name + '_place_grasp'

        # 放置时的姿态
        p.place_pose.header.frame_id = self.ref_frame
        p.place_pose.pose = frame_to_pose(target_transform)

        # 夹持器张开的姿态
        p.post_place_posture = self.JointTrajectory(open_place_posture, place_timeout)

        # 接近与撤离目标的参数
        p.pre_place_approach = pre_place_approach
        p.post_place_retreat = post_place_retreat

        # 其他参数
        p.allowed_touch_objects = [target_name]

        # self.cmd.set_support_surface_name('ground')

        result = False
        attempts = 0
        while result != MoveItErrorCodes.SUCCESS and attempts < max_attempts:
            result = self.cmd.place(target_name, p)
            rospy.sleep(0.01)
            attempts += 1
    # ---- absolute transform ---- #

    def to_target(self, name):
        self.cmd.set_named_target(name)
        self.cmd.go(wait=self.wait)

    def commit(self):

        plan = None  # 规划的路径
        fraction = 0.0  # 路径规划覆盖率
        attempts = 0  # 已经尝试次数

        while fraction < 1.0 and attempts < self.max_tries:
            (plan, fraction) = self.cmd.compute_cartesian_path(
                self.way_points,  # 路点列表
                0.001,  # 步进值
                0.0,  # 跳跃阈值
                True  # 避障规划
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
                          str(self.max_tries) + " attempts. Give up Cartesian mode.")
            for point in self.way_points:
                self.to_transform(point, cartesian=False)

        self.way_points = []

    def to_transform(self, xyz, rpy=None, cartesian=None, auto_commit=None):
        pose = None

        if isinstance(xyz, Frame):
            pose = frame_to_pose(xyz)
        elif isinstance(xyz, Pose):
            pose = xyz
        elif (isinstance(xyz, tuple) or isinstance(xyz, list)) and (isinstance(rpy, tuple) or isinstance(rpy, list)):
            pos = Vector(xyz[0], xyz[1], xyz[2])
            rot = Rotation.RPY(rpy[0], rpy[1], rpy[2])
            pose = frame_to_pose(Frame(rot, pos))

        if cartesian is None:
            cartesian = self.cartesian

        if auto_commit is None:
            auto_commit = self.auto_commit

        if cartesian:
            if len(self.way_points) == 0:
                self.way_points.append(self.get_pose())
            self.way_points.append(pose)
            if auto_commit:
                self.commit()
        else:
            self.cmd.set_start_state_to_current_state()
            self.cmd.set_pose_target(pose, self.end_link)
            plan = self.cmd.plan()
            self.cmd.execute(plan)

    def to_translate(self, xyz, cartesian=None, auto_commit=None):
        cur = self.get_transform()
        cur.p = Vector(xyz[0], xyz[1], xyz[2])
        self.to_transform(cur, cartesian=cartesian, auto_commit=auto_commit)

    def to_rotate(self, rpy, cartesian=None, auto_commit=None):
        cur = self.get_transform()
        cur.M = Rotation.RPY(rpy[0], rpy[1], rpy[2])
        self.to_transform(cur, cartesian=cartesian, auto_commit=auto_commit)

    # ---- local transform ---- #

    def local_transform(self, transform, cartesian=None, auto_commit=None):
        self.to_transform(self.get_transform() * transform, cartesian=cartesian, auto_commit=auto_commit)

    def local_translate(self, xyz, cartesian=None, auto_commit=None):
        self.local_transform(Frame(Vector(xyz[0], xyz[1], xyz[2])), cartesian=cartesian, auto_commit=auto_commit)

    def local_rotate(self, rpy, cartesian=None, auto_commit=None):
        self.local_transform(Frame(Rotation.RPY(rpy[0], rpy[1], rpy[2])), cartesian=cartesian, auto_commit=auto_commit)

    # ---- global transform ---- #

    def global_transform(self, transform, cartesian=None, auto_commit=None):
        self.to_transform(transform * self.get_transform(), cartesian=cartesian, auto_commit=auto_commit)

    def global_translate(self, xyz, cartesian=None, auto_commit=None):
        self.global_transform(Frame(Vector(xyz[0], xyz[1], xyz[2])), cartesian=cartesian, auto_commit=auto_commit)

    def global_rotate(self, rpy, cartesian=None, auto_commit=None):
        self.global_transform(Frame(Rotation.RPY(rpy[0], rpy[1], rpy[2])), cartesian=cartesian, auto_commit=auto_commit)

    # ---- utils ---- #

    def GripperTranslation(self, direction, desired_distance, min_distance):
        # type: (tuple, float, float) -> GripperTranslation
        translation = GripperTranslation()
        translation.direction.header.frame_id = self.ref_frame
        translation.direction.vector.x = direction[0]
        translation.direction.vector.y = direction[1]
        translation.direction.vector.z = direction[2]
        translation.desired_distance = desired_distance
        translation.min_distance = min_distance
        return translation

    def JointTrajectory(self, position, time=0.5):
        # type: (float, float) -> JointTrajectory

        trajectory = JointTrajectory()
        trajectory.header.frame_id = self.ref_frame
        trajectory.joint_names = ['sh_finger1_joint', 'sh_finger2_joint']

        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0, 0]
        point.time_from_start = Duration.from_sec(time)

        trajectory.points = [point]

        # print(trajectory)
        return trajectory
