#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 
import numpy as np
from std_msgs.msg import Float64, Bool, String
from morai_msgs.msg import GetTrafficLightStatus
import json
from utills import check_timer

class PerTraffic:
    def __init__(self):
        print(f"PerTraffic start")
        rospy.init_node('per_traffic_node')
        self.init_pubSub()
        self.init_traffic()
        
    def init_pubSub(self):
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.CB_traffic_raw, queue_size=1)
        self.pub_is_go_traffic = rospy.Publisher('/is_to_go_traffic', Bool, queue_size=1)
    def init_traffic(self):
        self.signal = 0
        self.prev_signal = 0
        self.traffic_msg = None
    def init_timer(self):
        self.check_timer = check_timer.CheckTimer("PerTraffic")
    def CB_traffic_raw(self, msg):
        self.traffic_msg = msg
        if self.traffic_msg.trafficLightIndex == "SN000005":
            self.signal = self.traffic_msg.trafficLightStatus
            if self.prev_signal != self.signal:
                self.prev_signal = self.signal
        self.processing()
                
    def pub_check_traffic(self):
        is_go = self.signal in [16, 33]
        self.pub_is_go_traffic.publish(Bool(data=is_go))

    def processing(self):
        try:
            self.pub_check_traffic()
        except Exception as e:
            pass

            
if __name__ == '__main__':
    node = PerTraffic()
    rospy.spin()
