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

class CtrlMotor:
    def __init__(self):
        # ROS 노드 초기화
        print(f"CtrlMotor start")
        rospy.init_node('ctrl_motor_node', anonymous=True)
        # pub 정리
        # 모터 pub 생성
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        rospy.Subscriber("/commands/motor/ctrl", Float64, self.motor_CB)
        self.motor_speed_CB = Float64()
        self.motor_cmd_msg_pub = Float64()
        self.check_timer = check_timer.CheckTimer("CtrlServo")
        # 퍼블리시 주기 설정 (10Hz)
        
    def motor_CB(self,msg:Float64):
        self.check_timer.start()
        self.motor_speed_CB = msg.data
        self.motor_pub_func()
        
    def motor_pub_func(self):
        self.motor_cmd_msg_pub.data = self.motor_speed_CB * 300
        self.motor_pub.publish(self.motor_cmd_msg_pub)
        print(f"speed {self.motor_cmd_msg_pub.data}")
        self.check_timer.check()
if __name__ == '__main__':
    try:
        node = CtrlMotor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass