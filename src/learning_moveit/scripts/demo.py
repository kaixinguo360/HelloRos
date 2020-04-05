#!/usr/bin/python2
# coding=utf-8
from arm import Arm
from builder import Builder
from scene import Scene
from utils import *

init('moveit_demo')

scene = Scene()
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))

arm = Arm()
arm.cartesian = True

builder = Builder(arm, scene)
builder.load_from_path(builder.get_path() + '/config/simple.json')
# builder.show_target()
# rospy.sleep(5)
builder.prepare_scene()
builder.build()

