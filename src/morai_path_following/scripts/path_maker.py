#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
import os
from math import sqrt
from morai_msgs.msg import EgoVehicleStatus

class pathMaker:
    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.is_status = False

        # 상대 경로 설정
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('morai_path_following')
        file_path = os.path.join(pkg_path, 'path', 'morai_ego_path.txt')

        self.f = open(file_path, 'w')

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_status:
                self.path_make()
            rate.sleep()

        self.f.close()

    def path_make(self):
        x = self.status_msg.position.x
        y = self.status_msg.position.y
        distance = sqrt((x - self.prev_x) ** 2 + (y - self.prev_y) ** 2)

        if distance > 0.3:
            data = '{0:.3f}\t{1:.3f}\n'.format(x, y)
            self.f.write(data)
            self.prev_x = x
            self.prev_y = y
            print("write :", x, y)

    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

if __name__ == '__main__':
    try:
        pathMaker()
    except rospy.ROSInternalException:
        pass


