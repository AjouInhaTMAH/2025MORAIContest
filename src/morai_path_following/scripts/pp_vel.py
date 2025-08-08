#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Path

class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller', anonymous=True)

        # Subscribers
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber('/local_path', Path, self.path_callback)

        # Publishers (변경된 부분)
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

        # Parameters
        # self.lfd = 5.0  # look-forward distance
        self.lfd = 0.5  # look-forward distance 테스트로 낮춰봄, 
        # **
        self.vehicle_length = 2.6
        self.target_velocity = 20 / 3.6  # 20 km/h in m/s

        self.status = None
        self.path = None

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.status and self.path:
                self.control()
            rate.sleep()

    def ego_callback(self, msg):
        self.status = msg

    def path_callback(self, msg):
        self.path = msg

    def control(self):
        # 현재 위치
        ego_x = self.status.position.x
        ego_y = self.status.position.y

        # 전방 추종 점 탐색
        found = False
        for pose in self.path.poses:
            # yaw = self.status.heading * np.pi / 180  # deg → rad
            # 모라이에서의 조향값은 -1~1 로 표현된다.
            yaw = self.status.heading * math.pi / 180.0
            dx = pose.pose.position.x - ego_x
            dy = pose.pose.position.y - ego_y

            # 차량 좌표계로 변환
            local_x =  math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            
            #dist = math.sqrt(dx ** 2 + dy ** 2)
            dist = math.sqrt(local_x**2 + local_y**2)

            if dist >= self.lfd and local_x > -1:  # 전방에 있는 점만 선택
                target_x = local_x
                target_y = local_y
                found = True
                break

        if found:
            # theta = math.atan2(target_y, target_x)
            # steering_angle = math.atan2(2 * self.vehicle_length * math.sin(theta), self.lfd)
            # max_steering_angle = np.radians(25)  # 예: ±25도 제한
            # steering_angle = max(-max_steering_angle, min(steering_angle, max_steering_angle))
            #-------------------------------------------------------------------------------------
            # sunsunsunsunsunsunsunsunsunsunsunsunsunsun
            theta = math.atan2(target_y, target_x)
            steering_angle = math.atan2(2 * self.vehicle_length * math.sin(theta), self.lfd)
            steering_norm = - (steering_angle / 3) + 0.5
            #-------------------------------------------------------------------------------------
        else:
            steering_angle = 0.0
            # rospy.logwarn("No look-forward point found.")
            rospy.logwarn("No look-forward point found.")
            steering_norm = 0.0  # ✅ 꼭 정의해줘야 함

        # 속도 제어 (단순 목표 속도 유지)
        current_vel = self.status.velocity.x
        accel = self.target_velocity - current_vel
        speed_cmd = max(self.target_velocity, 0.0)  # 기본 속도 유지

        # 퍼블리시
        #self.steer_pub.publish(Float64(data=steering_angle))
        self.steer_pub.publish(Float64(data=steering_norm))  # ✅ 0.0 ~ 1.0 범위로 정규화된 값
        self.speed_pub.publish(Float64(data=speed_cmd * 300))

        # 디버그 출력
        rospy.loginfo_throttle(1.0, f"[PurePursuit] speed: {speed_cmd:.2f} m/s | steering: {steering_angle:.3f} rad")

if __name__ == '__main__':
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
