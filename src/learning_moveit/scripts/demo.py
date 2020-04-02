#!/usr/bin/python2
# coding=utf-8
import rospy
from arm import Arm
from scene import Scene
from utils import *


pi = 3.1415926


def global_test():
    for z in range(6, 12, 1):
        arm.to_transform((0, 0.5, z * 0.1), (0, pi, 0))
        arm.global_translate((0.1, 0, 0))
        arm.global_translate((0, 0.1, 0))
        arm.global_translate((0, 0, 0.1))
        arm.global_rotate((0, 0, pi/16))
        arm.global_rotate((0, pi/16, 0))
        arm.global_rotate((pi/16, 0, 0))


def local_test():
    for z in range(6, 12, 1):
        arm.to_transform((0, 0.5, z * 0.1), (0, pi, 0))
        arm.local_translate((0.1, 0, 0))
        arm.local_translate((0, 0.1, 0))
        arm.local_translate((0, 0, 0.1))
        arm.local_rotate((0, 0, pi))
        arm.local_rotate((0, pi, 0))
        arm.local_rotate((pi, 0, 0))


brick_size=(0.4, 0.1, 0.2)
bricks = []


def add_brick(xyz, rpy=(pi, 0, 0)):
    transform = build_frame(xyz, rpy)
    scene.add_box(
        'brick' + str(len(bricks)),
        brick_size,
        transform,
        (1, 1, 0, 1)
    )
    bricks.append(transform)


def add_bricks(count):
    for x in range(-1, 2, 2):
        for y in range(-3, 2, 1):
            add_brick((0.5 * x, y * 0.3, brick_size[2]/2))
            if len(bricks) >= count:
                return
    print(bricks)


hand_offset = build_frame((0, 0, -0.2), (0, 0, pi/2))


def from_point_to_point(frame1, frame2):

    # pick
    arm.to_transform(frame1 * build_frame((0, 0, -0.2)) * hand_offset, cartesian=False)
    arm.to_transform(frame1 * hand_offset)
    arm.to_transform(frame1 * build_frame((0, 0, -0.2)) * hand_offset)

    # place
    arm.to_transform(build_frame((0, 0.6, 0.4), frame2.M.GetRPY()) * hand_offset, cartesian=False)
    arm.to_transform(build_frame((0, -0.2, 0.2)) * frame2 * hand_offset, auto_commit=False)
    arm.to_transform(build_frame((0, 0, 0.2)) * frame2 * hand_offset, auto_commit=False)
    arm.to_transform(frame2 * hand_offset)
    arm.to_transform(frame2 * build_frame((0, 0, -0.4)) * hand_offset)


init('moveit_demo')

scene = Scene()
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))

add_bricks(10)

arm = Arm()
arm.cartesian = True

while True:
    for transform in bricks:
        from_point_to_point(transform, build_frame((0.1, 1, 0.5), (-pi/2, 0, 0)))
    pass

