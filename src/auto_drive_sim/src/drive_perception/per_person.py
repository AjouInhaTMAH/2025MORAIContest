#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
import torch
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from auto_drive_sim.msg import PersonBBox  # ← 커스텀 메시지에 confidence 필드 포함
from time import *
from utills import check_timer
import json

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)

class PerPerson:
    def __init__(self):
        print(f"PerPerson start")
        rospy.init_node("per_person_node", anonymous=True)
        self.init_pubSub()
        self.init_model()
        self.init_timer()
    def init_pubSub(self):
        self.sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.CB_cam_raw, queue_size=1)
        self.pub = rospy.Publisher("/person_bbox", String, queue_size=1)
    def init_model(self):
        self.bridge = CvBridge()
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s", force_reload=False)
        self.model.conf = 0.5  # 신뢰도 기준 설정
        self.model.to('cuda').eval().half()
        self.img = None
        
         # --------------- 동적 장애물 상태 ---------------
         # __init__ 안 어디 적당히:
        self.frame_w = rospy.get_param("~frame_width", 640)
        self.frame_h = rospy.get_param("~frame_height", 480)
        self.center_margin_ratio = rospy.get_param("~center_margin_ratio", 0.25)  # 중앙폭 20%
        self.warn_height_ratio = rospy.get_param("~warn_height_ratio", 0.7)      # 15% 이상 감속
        self.stop_height_ratio = rospy.get_param("~stop_height_ratio", 0.15)      # 20% 이상 정지
        self.dynamic_obs_timeout = rospy.get_param("~dynamic_obs_timeout", 0.5)   # s
        self.dynamic_obs_flag = "none"  # 'none' | 'slow_flag' | 'stop_flag'
        self.dynamic_obs_timeout = rospy.get_param("~dynamic_obs_timeout", 0.5)  # [ADD] YOLO 타임아웃
        self.stop_hold_until = 0.0  # [ADD] 정지 유지(홀드) 타임스탬프
        
    def init_timer(self):
        self.check_timer = check_timer.CheckTimer("PerPerson")

    def CB_cam_raw(self, msg):
        self.img = msg
        self.processing()
    
    def pub_person_info(self,data):
        json_str = json.dumps(data)
        self.pub.publish(json_str)
    
    def detect_obstacle(self,data):
        center_x   = (data[0] + data[2]) / 2.0
        box_height = data[3] - data[1]
        
        # 중앙 영역 유지 (비율 기반)
        margin_half = self.frame_w * self.center_margin_ratio * 0.5
        center_min  = self.frame_w/2.0 - margin_half
        center_max  = self.frame_w/2.0 + margin_half
        in_center   = (center_min - 40 <= center_x <= center_max + 40)

        h_ratio = box_height / float(self.frame_h)

        # ★ 순서: stop 먼저 → slow
        if in_center and h_ratio >= self.stop_height_ratio:
            self.dynamic_obs_flag = "stop_flag"
            self.last_detected_time = rospy.get_time()
        elif in_center and h_ratio >= self.warn_height_ratio:
            self.dynamic_obs_flag = "slow_flag"
            self.last_detected_time = rospy.get_time()
        else:
            self.dynamic_obs_flag = "none"
        return self.dynamic_obs_flag
        # print(self.dynamic_obs_flag)
    
    def extract_dynamic_obs(self):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(self.img, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("❌ CV bridge error: %s", e)
            return

        # self.check_timer.start()
        results = self.model(img)
        # self.check_timer.check()
        detections = results.pandas().xyxy[0]  # pandas DataFrame
        self.detect_person_result = False
        for _, row in detections.iterrows():
            class_name = str(row['name'])  # YOLOv5는 보통 문자열 'person'
            conf = float(row['confidence'])

            if class_name == '2' or class_name.lower() == 'person':  # 둘 다 허용
                x1, y1, x2, y2 = map(float, [row['xmin'], row['ymin'], row['xmax'], row['ymax']])
                
                data = [x1, y1, x2, y2, conf]
                self.detect_person_result = self.detect_obstacle(data)

                # 🔍 시각화
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.putText(img, f"person {conf:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                break  # 첫 사람만
                
        # 출력 영상 표시
        self.pub_person_info(self.detect_person_result)
        cv2.imshow("YOLO Detection", img)
        cv2.waitKey(1)

    def processing(self):
        try:
            self.extract_dynamic_obs()
        except Exception as e:
            pass
if __name__ == '__main__':
    node = PerPerson()
    rospy.spin()
    