#!/usr/bin/python2
# coding=utf-8
import json
import rospy
from arm import Arm
from scene import Scene
from builder import Builder
from utils import *


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


init('moveit_demo')

scene = Scene()
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))

arm = Arm()
arm.cartesian = True

builder = Builder(arm, scene)
builder.load_from_path(builder.get_path() + '/config/demo.json')
builder.show_target()
rospy.sleep(5)
builder.prepare_scene()
builder.build()

