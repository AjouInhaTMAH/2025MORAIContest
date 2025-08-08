#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
import dec_mission_all
import threading

class DecMain:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('DecMainNode', anonymous=True)
        self.missionAll = dec_mission_all.DecMissionAll()
        
        # 📌 processing 함수 쓰레드로 실행
        self.missionAll_thread = threading.Thread(target=self.missionAll.processing)
        self.missionAll_thread.daemon = True  # 메인 종료 시 같이 종료되게 할 경우
        self.missionAll_thread.start()
        
if __name__ == '__main__':
    try:
        node = DecMain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass