#!/usr/bin/python2
# coding=utf-8
from arm_common import *


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


arm = Arm('moveit_ik_demo')
arm.cartesian = True

while True:
    local_test()

