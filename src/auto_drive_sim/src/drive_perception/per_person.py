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
        self.sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.CB_cam_raw)
        self.pub = rospy.Publisher("/person_bbox", String, queue_size=10)
    def init_model(self):
        self.bridge = CvBridge()
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s", force_reload=False)
        self.model.conf = 0.5  # 신뢰도 기준 설정
        self.img = None
    def init_timer(self):
        self.check_timer = check_timer.CheckTimer("PerPerson")

    def CB_cam_raw(self, msg):
        self.img = msg
    
    def pub_person_info(self,data):
        json_str = json.dumps(data)
        self.pub.publish(json_str)
        # print(f"pub {data}")
    
    def detect_obstacle(self,data):
        center_x   = (data[0] + data[2]) / 2.0
        box_height = data[3] - data[1]
        frame_width, frame_height = 640, 480
        margin = frame_width * 0.15 / 2.0
        center_min = frame_width/2.0 - margin
        center_max = frame_width/2.0 + margin
        HEIGHT_THRESHOLD = 0.2 * frame_height
        if (center_min <= center_x <= center_max) and (box_height > HEIGHT_THRESHOLD):
            return True
        else:
            return False
    
    def extract_dynamic_obs(self):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(self.img, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("❌ CV bridge error: %s", e)
            return

        results = self.model(img)
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
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            if self.img is not None:
                # self.check_timer.start()
                self.extract_dynamic_obs()
                self.img = None
                # self.check_timer.check()
                # print(f"time {time()-start}")
            rate.sleep()
if __name__ == '__main__':
    node = PerPerson()
    node.processing()   # spin 대신 processing 돌기