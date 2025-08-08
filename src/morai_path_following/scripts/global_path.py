#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import os

class GlobalPathPublisher:
    def __init__(self):
        rospy.init_node('global_path_publisher', anonymous=True)
        self.pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # rospkg로 패키지 내부 경로 지정
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('morai_path_following')
        file_path = os.path.join(pkg_path, 'path', 'morai_ego_path.txt')

        with open(file_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                x, y = map(float, line.strip().split())
                pose = PoseStamped()
                # 좌표계 주의 포인트 **
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                self.path_msg.poses.append(pose)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.path_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        GlobalPathPublisher()
    except rospy.ROSInterruptException:
        pass

