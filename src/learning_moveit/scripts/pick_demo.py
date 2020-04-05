#!/usr/bin/python2
# coding=utf-8

from arm import Arm
from scene import Scene
from utils import *

init('moveit_demo')

source = build_frame((-0.5, -0.5, 0.1), (pi, 0, 0))
target = build_frame((-0.5, 0.5, 0.2), (0, -pi/2, -pi/2))

scene = Scene()
scene.add_box('ground', (5, 5, 0.1), build_frame((0, 0, -0.05)))
scene.add_box('box', (0.4, 0.1, 0.2), source, (1, 1, 0, 1))

arm = Arm()
arm.cartesian = True

# arm.to_target('up')

arm.pick(
    'box',
    source * build_frame((0, 0, -0.16)),
    arm.GripperTranslation((0, 0, -1), 0.1, 0.05),
    arm.GripperTranslation((0, 0, 1), 0.1, 0.05),
    arm.GripperTranslation((0, 1, 0), 0.1, 0.05),
    0.09,
    0.06
)

arm.place(
    'box',
    target,
    arm.GripperTranslation((0, 0, -1), 0.1, 0.05),
    arm.GripperTranslation((0, -1, 0), 0.1, 0.05),
    0.09,
)
