#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 현재 위치 기준으로 /global_path에서 일부 구간을 /local_path로 퍼블리시하는 노드입니다.


import rospy
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus

class LocalPathGenerator:
    def __init__(self):
        rospy.init_node('local_path_generator', anonymous=True)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber('/global_path', Path, self.global_path_callback)
        self.pub = rospy.Publisher('/local_path', Path, queue_size=1)

        self.global_path = Path()
        self.x = 0.0
        self.y = 0.0
        self.is_ready = False
        self.local_path_size = 10

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.is_ready:
                local_path = Path()
                local_path.header.frame_id = 'map'

                closest_idx = -1
                min_dist = float('inf')
                for i, pose in enumerate(self.global_path.poses):
                    dist = sqrt(pow(self.x - pose.pose.position.x, 2) +
                                pow(self.y - pose.pose.position.y, 2))
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = i

                if closest_idx != -1:
                    for i in range(closest_idx, min(closest_idx + self.local_path_size, len(self.global_path.poses))):
                        # global_path에서 가져온 pose 복사
                        orig_pose = self.global_path.poses[i]
                        pose = PoseStamped()
                        pose.header.frame_id = 'map'  # ✅ 명시적 설정
                        pose.pose = orig_pose.pose    # pose 복사
                        local_path.poses.append(self.global_path.poses[i])

                self.pub.publish(local_path)
            rate.sleep()

    def ego_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.is_ready = True

    def global_path_callback(self, msg):
        self.global_path = msg

if __name__ == '__main__':
    try:
        LocalPathGenerator()
    except rospy.ROSInterruptException:
        pass


