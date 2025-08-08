#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
from std_msgs.msg import Int32
import math
from time import *
from utils import check_timer
from std_msgs.msg import String
import json

class PerCarNavigation:
    def __init__(self):
        print(f"PerCarNavigation start")
        rospy.init_node('per_car_nav_node')
        # zone 좌표 정의 (map 좌표 기준 -> /amcl_pose를 morai에서 실제로 뽑아옴, )
        self.zones = {
            1: (-0.02331218035026452, 10.6556),   # mission 2 & 3 영역
            2: (4.7866139, 4.34265),    # mission 5 영역
            3: (8.2325, 1.7338),     # mission 5 영역
            4: (10.1863, -0.9488),     # mission 5 영역
            # 401:(3.5279, 7.048),
            # 402:(4.0596, 7.0403),
            # 403:(4.6748, 6.52258),
            # 404:(4.8693, 6.2139),
        }
        self.zone_threshold = 0.5  # 허용 반경 (meters)
        rospy.Subscriber('/cur_pose', String, self.CB_currentPose_info)
        self.x = 0
        self.y = None
        self.w = 0
        self.vel = 0
        self.recent_zone = None
        self.zone_pub = rospy.Publisher('/lane_mode', Int32, queue_size=1)
        self.rate = rospy.Rate(40)
        self.init_timer()
    def init_timer(self):
        self.check_timer = check_timer.CheckTimer("PerCarNavigation")

    def CB_currentPose_info(self, msg):
        try:
            data = json.loads(msg.data)
            self.x = data.get("x")
            self.y = data.get("y")
            self.yaw = data.get("yaw")
            self.vel = data.get("vel")
        except json.JSONDecodeError as e:
            rospy.logwarn(f"JSON Decode Error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            
    def check_zones(self):
        x = self.x
        y = self.y
        start = time()
        for zone_id, (zx, zy) in self.zones.items():
            dist = math.hypot(x - zx, y - zy)

            if dist < self.zone_threshold:
                # if self.recent_zone != zone_id:
                rospy.loginfo(f"📍 현재 위치 zone {zone_id} 감지됨 (x={x:.2f}, y={y:.2f})")
                self.zone_pub.publish(Int32(zone_id))
                self.rate.sleep()
                self.recent_zone = zone_id
                return

        end = time()
        # print(f"time {end - start}")
        # zone 범위 벗어난 경우
        self.recent_zone = None
        
    def processing(self):
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            if self.y is not None:
                self.check_timer.start()
                self.check_zones()
                self.rate.sleep()
                # self.check_timer.check()
                self.y = None
            rate.sleep()
                
if __name__ == '__main__':
    node = PerCarNavigation()
    node.processing()   # spin 대신 processing 돌기