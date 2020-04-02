#!/usr/bin/python2
# coding=utf-8
import json
import rospkg

from PyKDL import Frame, Vector, Rotation
from geometry_msgs.msg import Pose

from utils import frame_to_pose, pose_to_frame, build_frame, pi


class Builder:
    arm = None
    scene = None

    config = None

    brick_size = (0.4, 0.1, 0.2)
    bricks = []

    base_offset = build_frame((0.1, 1, 0))
    hand_offset = build_frame((0, 0, -0.2), (0, 0, pi / 2))
    rp = rospkg.RosPack()

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
            self.from_point_to_point(self.bricks[index], target)

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

    def from_point_to_point(self, frame1, frame2):
        # pick
        self.arm.to_transform(frame1 * build_frame((0, 0, -0.2)) * self.hand_offset, cartesian=False)
        self.arm.to_transform(frame1 * self.hand_offset)
        self.arm.to_transform(frame1 * build_frame((0, 0, -0.2)) * self.hand_offset)

        # place
        self.arm.to_transform(build_frame((0, 0.6, 0.4), frame2.M.GetRPY()) * self.hand_offset, cartesian=False)
        self.arm.to_transform(build_frame((0, -0.2, 0.2)) * frame2 * self.hand_offset, auto_commit=False)
        self.arm.to_transform(build_frame((0, 0, 0.2)) * frame2 * self.hand_offset, auto_commit=False)
        self.arm.to_transform(frame2 * self.hand_offset)
        self.arm.to_transform(frame2 * build_frame((0, 0, -0.4)) * self.hand_offset)