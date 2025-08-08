#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
import rospy
from std_msgs.msg import Float64
from utils import check_timer

class CtrlServo:
    def __init__(self):
        # ROS 노드 초기화
        print(f"CtrlServo start")
        rospy.init_node('ctrl_servo_node', anonymous=True)
        # pub 정리
        # 모터 pub 생성
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        rospy.Subscriber("/commands/servo/ctrl", Float64, self.servo_CB)
        self.servo_steer_CB = Float64()
        self.servo_cmd_msg_pub = Float64()
        self.check_timer = check_timer.CheckTimer("CtrlServo")
        # 퍼블리시 주기 설정 (10Hz)
        
    def servo_CB(self,msg:Float64):
        self.check_timer.start()
        self.servo_steer_CB = msg.data
        self.servo_pub_func()
        
    def servo_pub_func(self):
        self.servo_cmd_msg_pub.data = ((self.servo_steer_CB / 19.5 + 1)) /2
        self.servo_pub.publish(self.servo_cmd_msg_pub)
        print(f"servo {self.servo_cmd_msg_pub.data}")
        self.check_timer.check()
        
if __name__ == '__main__':
    try:
        node = CtrlServo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
