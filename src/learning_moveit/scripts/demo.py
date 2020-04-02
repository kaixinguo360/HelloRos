#!/usr/bin/python2
# coding=utf-8
import rospy
from arm import Arm
from scene import Scene
from utils import *


def global_test():
    for z in range(6, 12, 1):
        arm.to_transform((0, 0.5, z * 0.1), (0, 3.1415926, 0))
        arm.global_translate((0.1, 0, 0))
        arm.global_translate((0, 0.1, 0))
        arm.global_translate((0, 0, 0.1))
        arm.global_rotate((0, 0, 3.1415926/16))
        arm.global_rotate((0, 3.1415926/16, 0))
        arm.global_rotate((3.1415926/16, 0, 0))


def local_test():
    for z in range(6, 12, 1):
        arm.to_transform((0, 0.5, z * 0.1), (0, 3.1415926, 0))
        arm.local_translate((0.1, 0, 0))
        arm.local_translate((0, 0.1, 0))
        arm.local_translate((0, 0, 0.1))
        arm.local_rotate((0, 0, 3.1415926))
        arm.local_rotate((0, 3.1415926, 0))
        arm.local_rotate((3.1415926, 0, 0))


brick_size=(0.4, 0.1, 0.2)
bricks = []


def add_brick(xyz, rpy=(0, 0, 0)):
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
        for y in range(-2, 3, 1):
            add_brick((0.5 * x, y * 0.4, brick_size[2]/2))
            if len(bricks) >= count:
                return
    print(bricks)


init('moveit_demo')

scene = Scene()
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))

add_bricks(10)

arm = Arm()

while True:
    local_test()
    pass

