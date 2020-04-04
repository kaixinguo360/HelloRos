#!/usr/bin/python2
# coding=utf-8
import json
import rospkg
from rospkg import RosPack

from PyKDL import Frame

from arm import Arm
from scene import Scene
from utils import build_frame, pi


class Builder:
    arm = None  # type: Arm
    scene = None  # type: Scene

    config = None  # type: list

    brick_size = (0.4, 0.1, 0.2)  # type: tuple
    bricks = []  # type: list

    base_offset = build_frame((0.1, 1, 0))  # type: Frame
    hand_offset = build_frame((0, 0, -0.16), (0, 0, 0))  # type: Frame
    rp = rospkg.RosPack()  # type: RosPack

    def __init__(self, arm=None, scene=None):
        self.arm = arm
        self.scene = scene

    # ---- load ---- #

    def load_from_path(self, config_path):
        with open(config_path, 'r') as config_file:
            config = json.load(config_file)
            self.load(config)

    def load(self, config):
        self.config = config
        self.sort_config()

    # ---- scene ---- #

    def prepare_scene(self):
        self.reset_scene()
        for y in range(1, -4, -1):
            for x in range(-1, 2, 2):
                self.add_brick((0.5 * x, y * 0.3, self.brick_size[2] / 2))
                if len(self.bricks) >= len(self.config):
                    return
        for y in range(1, -3, -1):
            for x in range(-1, 2, 2):
                self.add_brick((0.8 * x, -0.15 + y * 0.3, self.brick_size[2] / 2))
                if len(self.bricks) >= len(self.config):
                    return

    def show_target(self):
        self.reset_scene()
        for target in self.config:
            target = self.base_offset * build_frame(target['xyz'], target['rpy'])
            self.add_brick((
                target.p.x(), target.p.y(), target.p.z()
            ), target.M.GetRPY())

    def reset_scene(self):
        while len(self.bricks) > 0:
            self.bricks.pop()
            self.scene.remove_object('brick' + str(len(self.bricks)))

    # ---- build ---- #

    def build(self):
        for index in range(0, len(self.config), 1):
            config = self.config[index]
            target = self.base_offset * build_frame(config['xyz'], config['rpy'])
            self.from_point_to_point(self.bricks[index], target, index)

    # ---- utils ---- #

    def get_path(self):
        return self.rp.get_path('learning_moveit')

    def sort_config(self):
        self.config.sort(key=lambda x: x['xyz'][2]*10000 + -x['xyz'][1]*100 + x['xyz'][0])

    def add_brick(self, xyz, rpy=(pi, 0, 0)):
        transform = build_frame(xyz, rpy)
        self.scene.add_box(
            'brick' + str(len(self.bricks)),
            self.brick_size,
            transform,
            (1, 1, 0, 1)
        )
        self.bricks.append(transform)

    def from_point_to_point(self, frame1, frame2, index):

        self.arm.pick(
            'brick' + str(index),
            frame1 * self.hand_offset,
            self.arm.GripperTranslation((0, 0, -1), 0.2, 0.1),
            self.arm.GripperTranslation((0, 0, 1), 0.2, 0.1),
            self.arm.GripperTranslation((0, 0, 1), 0.2, 0.1),
            0.09,
            (self.brick_size[1] + 0.02) / 2,
            2
        )

        self.arm.place(
            'brick' + str(index),
            frame2
        )

        # self.arm.pick('brick' + str(index), frame1 * self.hand_offset)
        # # pick
        # self.arm.to_transform(frame1 * build_frame((0, 0, -0.2)) * self.hand_offset, cartesian=False)
        # self.arm.to_transform(frame1 * self.hand_offset)
        # rospy.sleep(1)
        # self.arm.pick('brick' + str(index), frame1 * self.hand_offset)
        # self.close()
        # rospy.sleep(1)
        # self.arm.to_transform(frame1 * build_frame((0, 0, -0.2)) * self.hand_offset)
        #
        # # xxx
        # self.arm.to_transform(build_frame((0, 0.6, 0.4), frame2.M.GetRPY()) * self.hand_offset, cartesian=False)
        #
        # # place
        # self.arm.to_transform(build_frame((0, -0.2, 0.2)) * frame2 * self.hand_offset, auto_commit=False)
        # self.arm.to_transform(build_frame((0, 0, 0.2)) * frame2 * self.hand_offset, auto_commit=False)
        # self.arm.to_transform(frame2 * self.hand_offset)
        # rospy.sleep(1)
        # self.open()
        # self.place(index, frame2)
        # self.arm.global_translate((0, 0, -0.2))
