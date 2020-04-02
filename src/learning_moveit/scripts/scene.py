#!/usr/bin/python2
# coding=utf-8
import rospy

from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

from utils import frame_to_pose


class Scene:
    interface = None
    publisher = None
    ref_frame = ''

    colors = dict()
    default_color = (0.75, 0.75, 0.75, 1)

    def __init__(self, ref_frame='base_link'):
        self.interface = PlanningSceneInterface()
        self.publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.ref_frame = ref_frame
        rospy.sleep(1)

    def destroy(self):
        pass

    # ---- add object ---- #

    def add_box(self, name, size, transform, rgba=None):
        if rgba is None:
            rgba = self.default_color

        self.interface.remove_world_object(name)

        pose = PoseStamped()
        pose.header.frame_id = self.ref_frame
        pose.pose = frame_to_pose(transform)

        self.interface.add_box(name, pose, size)
        self.wait_state_update(name)

        self.set_color(name, rgba)
        self.send_colors()

    # ---- utils ---- #

    def set_color(self, name, rgba):
        color = ObjectColor()
        color.id = name
        color.color.r = rgba[0]
        color.color.g = rgba[1]
        color.color.b = rgba[2]
        color.color.a = rgba[3]
        self.colors[name] = color

    def send_colors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.publisher.publish(p)

    def wait_state_update(self, name, is_known=True, is_attached=False, timeout=4):

        start = rospy.get_time()
        current = rospy.get_time()
        while (current - start < timeout) and not rospy.is_shutdown():

            attached_objects = self.interface.get_attached_objects([name])
            cur_is_attached = len(attached_objects.keys()) > 0

            cur_is_known = name in self.interface.get_known_object_names()

            if (cur_is_attached == is_attached) and (cur_is_known == is_known):
                rospy.loginfo('Added')
                return True

            rospy.sleep(0.1)
            current = rospy.get_time()
            rospy.loginfo('start=   ' + str(start))
            rospy.loginfo('current= ' + str(current))
            rospy.loginfo('Trying... ' + str(current - start))

        rospy.loginfo('Failed')
        return False
