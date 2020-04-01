#!/usr/bin/python2
# coding=utf-8
from arm_common import Arm, Frame, Vector, Rotation


def to_translate(xyz):
    print('----------')
    print('arm.set_to_translate('
          + str(xyz[0]) + ', '
          + str(xyz[1]) + ', '
          + str(xyz[2]) + ')')
    arm.to_translate(xyz[0], xyz[1], xyz[2])
    print('done')


def to_transform(xyz, rpy):
    print('----------')
    print('arm.set_to_transform()')
    print('xyz=('
          + str(xyz[0]) + ', '
          + str(xyz[1]) + ', '
          + str(xyz[2]) + ')')
    print('rpy=('
          + str(rpy[0]) + ', '
          + str(rpy[1]) + ', '
          + str(rpy[2]) + ')')
    pos = Vector(xyz[0], xyz[1], xyz[2])
    rot = Rotation.RPY(rpy[0], rpy[1], rpy[2])
    arm.to_transform(Frame(rot, pos))
    print('done')


def global_test():
    for z in range(6, 12, 1):
        to_transform((0, 0.5, z * 0.1), (0, 3.1415926, 0))
        arm.global_translate((0.1, 0, 0))
        arm.global_translate((0, 0.1, 0))
        arm.global_translate((0, 0, 0.1))
        arm.global_rotate((0, 0, 3.1415926/16))
        arm.global_rotate((0, 3.1415926/16, 0))
        arm.global_rotate((3.1415926/16, 0, 0))


def local_test():
    for z in range(6, 12, 1):
        to_transform((0, 0.5, z * 0.1), (0, 3.1415926, 0))
        arm.local_translate((0.1, 0, 0))
        arm.local_translate((0, 0.1, 0))
        arm.local_translate((0, 0, 0.1))
        arm.local_rotate((0, 0, 3.1415926))
        arm.local_rotate((0, 3.1415926, 0))
        arm.local_rotate((3.1415926, 0, 0))

arm = Arm('moveit_ik_demo')

while True:
    local_test()

