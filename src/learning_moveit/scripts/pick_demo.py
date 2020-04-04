#!/usr/bin/python2
# coding=utf-8

from arm import Arm
from scene import Scene
from utils import *

init('moveit_demo')

position = build_frame((-0.5, -0.5, 0.2))

scene = Scene()
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))
scene.add_box('box', (0.02, 0.02, 0.1), position, (1,1,0,1))

arm = Arm()
arm.cartesian = True

arm.pick(
    'box',
    position * build_frame((0.1, 0, 0), (0, -pi/2, 0)),
    arm.GripperTranslation((-1, 0, 0), 0.1, 0.05),
    arm.GripperTranslation((1, 0, 0), 0.1, 0.05),
    arm.GripperTranslation((1, 0, 0), 0.2, 0.05),
    0.09,
    0.02
)

arm.place(
    'box',
    position,
)

