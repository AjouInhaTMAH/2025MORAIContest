#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Path

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller', anonymous=True)

        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber('/local_path', Path, self.path_callback)

        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

        self.vehicle_length = 2.5
        self.lfd = 1.0
        self.target_velocity = 5.0  # m/s 고정 속도

        self.current_vel = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.path = None
        self.is_path = False
        self.is_status = False

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.is_path and self.is_status:
                self.control()
            else:
                rospy.logwarn_throttle(2.0, "[PurePursuit] Waiting for path and ego status...")
            self.is_path = False
            self.is_status = False
            rate.sleep()

    def ego_callback(self, msg):
        self.is_status = True
        self.current_x = msg.position.x
        self.current_y = msg.position.y
        self.current_yaw = msg.heading  # degree
        print(f"current_x;{self.current_x}, current_y:{self.current_y}")

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg
        

    def control(self):
        lfd = self.lfd
        found = False

        for pose in self.path.poses:
            dx = pose.pose.position.x - self.current_x
            dy = pose.pose.position.y - self.current_y
            
            print(f"path : { pose.pose.position.x} : { pose.pose.position.y}")
            dist = math.hypot(dx, dy)

            if dist > lfd:
                target_x = dx
                target_y = dy
                found = True
                break

        if found:
            # heading은 degree로 들어오므로 rad로 변환
            yaw_rad = math.radians(self.current_yaw)

            # 차량 기준 좌표계 변환
            local_x =  math.cos(yaw_rad) * target_x + math.sin(yaw_rad) * target_y
            local_y = -math.sin(yaw_rad) * target_x + math.cos(yaw_rad) * target_y

            theta = math.atan2(local_y, local_x)
            steering_angle = math.atan2(2 * self.vehicle_length * math.sin(theta), lfd)

            # 정규화
            max_steering_rad = math.radians(35)
            steering_norm = max(-1.0, min(steering_angle / max_steering_rad, 1.0))

            self.steer_pub.publish(Float64(data=steering_norm))
            self.speed_pub.publish(Float64(data=self.target_velocity * 300))

            rospy.loginfo_throttle(1.0, f"[PurePursuit] speed: {self.target_velocity:.2f} m/s | steering: {steering_angle:.3f} rad")
        else:
            rospy.logwarn("No look-forward point found.")
            self.steer_pub.publish(Float64(data=0.0))
            self.speed_pub.publish(Float64(data=0.0))

if __name__ == '__main__':
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
