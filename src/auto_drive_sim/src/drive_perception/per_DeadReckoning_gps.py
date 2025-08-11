#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
import rospy
import math
import json
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from utills import check_timer

class DeadReckoningNode:
    def __init__(self):
        print(f"DeadReckoningNode start")
        rospy.init_node('dead_reckoning_node')
        self.init_pubSub()
        self.init_coordinate()
        self.init_timer()
        
    def init_pubSub(self):
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.CB_amcl_raw)
        rospy.Subscriber('/imu', Imu, self.CB_imu_raw)
        rospy.Subscriber('/sensors/core', VescStateStamped, self.CB_vel_raw)
        self.cur_pose_pub = rospy.Publisher('/cur_pose', String, queue_size=1)
    def init_coordinate(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.imu_yaw = 0.0
        self.vel = 0.0
        self.last_time = rospy.Time.now()
        self.initialized = False
        self.rpm = 0
    def init_timer(self):
        self.check_timer = check_timer.CheckTimer("DeadReckoningNode")
        
    def CB_amcl_raw(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = math.degrees(yaw_rad)
        self.last_time = rospy.Time.now()
        self.initialized = True
        # rospy.loginfo(f"[AMCL] 위치 초기화: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.1f}")
    def CB_imu_raw(self, msg):
        q = msg.orientation
        _, _, yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = math.degrees(yaw_rad)
    def CB_vel_raw(self, msg):
        self.rpm = msg.state.speed
        # self.check_time.start()

        # rospy.loginfo(f"[DR] x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.1f} vel={self.vel:.2f}")
        # self.check_time.check()

    def pub_cur_pose_info(self):
        # JSON 메시지 생성 및 publish
        msg_dict = {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 2),
            "vel": round(self.vel, 2)
        }
        msg_str = json.dumps(msg_dict)
        self.cur_pose_pub.publish(msg_str)
        
    def guess_coordinate(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        self.vel = self.rpm * 0.0013577451514397 * 1000 / (2 * math.pi * 60)

        yaw_rad = math.radians(self.imu_yaw)
        vx = self.vel * math.cos(yaw_rad)
        vy = self.vel * math.sin(yaw_rad)

        dx = dt * vx / 3.6
        dy = dt * vy / 3.6

        self.x += dx
        self.y += dy
        self.yaw = self.imu_yaw
     
    def processing(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if not self.initialized:
                continue
            self.guess_coordinate()
            self.pub_cur_pose_info()
            rate.sleep()
            
if __name__ == '__main__':
    node = DeadReckoningNode()
    node.processing()