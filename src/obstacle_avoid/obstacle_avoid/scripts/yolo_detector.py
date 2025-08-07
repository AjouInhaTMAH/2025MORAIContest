#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import torch
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from obstacle_avoid.msg import PersonBBox  # ← 커스텀 메시지에 confidence 필드 포함
from std_msgs.msg import Int32

class YOLOv5Detector:
    def __init__(self):
        rospy.init_node("yolo_v5_detector", anonymous=True)

        # ✅ YOLOv5 모델 로드 (best.pt)
        model_path = "/root/wego_ws/src/obstacle_avoid/yolo/best.pt"
        self.model = torch.hub.load("ultralytics/yolov5", "custom", path=model_path, force_reload=False)
        self.model.conf = 0.5  # 신뢰도 기준 설정

        # ROS 설정
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.pub = rospy.Publisher("/person_bbox", PersonBBox, queue_size=10)


    def image_callback(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("❌ CV bridge error: %s", e)
            return

        results = self.model(img)
        detections = results.pandas().xyxy[0]  # pandas DataFrame

        for _, row in detections.iterrows():
            class_name = str(row['name'])
            conf = float(row['confidence'])

            x1, y1, x2, y2 = map(float, [row['xmin'], row['ymin'], row['xmax'], row['ymax']])
            
            # 🔍 시각화
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(img, label, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 사람일 때만 퍼블리시
            if class_name == '2' or class_name.lower() == 'person':
                bbox_msg = PersonBBox()
                bbox_msg.xmin = x1
                bbox_msg.ymin = y1
                bbox_msg.xmax = x2
                bbox_msg.ymax = y2
                bbox_msg.confidence = conf
                self.pub.publish(bbox_msg)


        # 출력 영상 표시
        cv2.imshow("YOLO Detection", img)
        cv2.waitKey(1)


def main():
    try:
        detector = YOLOv5Detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
