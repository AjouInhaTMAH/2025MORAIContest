#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32
import math


class CarNavigation:
    def __init__(self):
        rospy.init_node('car_nav', anonymous=True)

        # zone 좌표 정의 (map 좌표 기준 -> /amcl_pose를 morai에서 실제로 뽑아옴, )
        self.zones = {
            # 1: (1.21967452, 9.5353531578),   # A 영역
            1: ( 0.226296941, 8.937600630), 
            2: (4.7866139, 4.34265),    # B 영역
            3: (7.928031, 1.50644),     # C 영역
        }
        self.zone_threshold = 1.0  # 허용 반경 (meters)

        self.current_pose = None
        self.recent_zone = None
        self.zone_pub = rospy.Publisher('/lane_mode', Int32, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        rospy.loginfo("✅ car_navigation 노드 시작")
        self.run()

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose:
                self.check_zones()
            rate.sleep()

    def check_zones(self):
        x = self.current_pose.position.x
        y = self.current_pose.position.y

        for zone_id, (zx, zy) in self.zones.items():
            dist = math.hypot(x - zx, y - zy)

            if dist < self.zone_threshold:
                if self.recent_zone != zone_id:
                    rospy.loginfo(f"📍 현재 위치 zone {zone_id} 감지됨 (x={x:.2f}, y={y:.2f})")
                    print(f"📍 현재 위치 zone {zone_id} 감지됨 (x={x:.2f}, y={y:.2f}")
                    self.zone_pub.publish(Int32(zone_id))
                    self.recent_zone = zone_id
                return

        # zone 범위 벗어난 경우
        self.recent_zone = None


if __name__ == '__main__':
    try:
        CarNavigation()
    except rospy.ROSInterruptException:
        pass


'''
계속해서 amcl_pose를 구독하면서 현재 위치 추적.

특정 pose 영역(A, B, C 등)에 도달하면:

내부 nav_pose와 비교 → 오차가 충분히 작으면
차선 추종 알고리즘에게 "지금은 왼쪽 차선 따라가라"는 트리거 메시지를 보냄

이유: 카메라는 여러 차선을 인식하므로, 특정 구간에서는 명확한 차선 추종 ROI 지시가 필요함.
'''