#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
import per_camera
import threading

class PerMain:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('PerMainNode', anonymous=True)
        self.camera = per_camera.PerCamera()
        
        # 📌 processing 함수 쓰레드로 실행
        self.camera_thread = threading.Thread(target=self.camera.processing)
        self.camera_thread.daemon = True  # 메인 종료 시 같이 종료되게 할 경우
        self.camera_thread.start()
        
if __name__ == '__main__':
    try:
        node = PerMain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass