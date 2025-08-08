#!/usr/bin/env python3
import rospy
import roslibpy
import sys
import os
import logging
import threading
import signal
import numpy as np

from std_msgs.msg import Int16MultiArray
from morai_msgs.msg import ObjectStatusList, ObjectStatus, EgoVehicleStatus

# Twisted/roslibpy 로그 최소화
logging.getLogger('roslibpy').setLevel(logging.CRITICAL)
logging.getLogger('twisted').setLevel(logging.CRITICAL)

RECONNECT_INTERVAL = 1  # 재연결 시도 간격 (초)

class ObjectTopicProcessor:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client = None
        self.connected = False
        self.should_run = True
        self.lock = threading.Lock()
        
        # Object_topic 관련 변수들
        self.obj_msg_ = ObjectStatusList()
        self.obj_msg_.num_of_pedestrian = 1
        self.obj_msg_.num_of_obstacle = 2

        self.ped_data_ = ObjectStatus()
        self.obj_1_data_ = ObjectStatus()
        self.obj_2_data_ = ObjectStatus()

        # Ego_topic 관련 변수들
        self.obj_reach_ = Int16MultiArray()
        self.obj_reach_.data = [0,0,0]
        self.max_distance_ = 1.2
        # self.prev_time_ = rospy.Time.now()
        self.reach_flag_ = False
        self.goal_free_time_ = rospy.Time.now()
        self.intarray_publisher = None
        self.intarray_topic = '/delivery_check'
        self.intarray_type = 'std_msgs/Int16MultiArray'

        # ROS 구독자들
        self.object_subscriber = None
        self.ego_subscriber = None
        
        # roslibpy 발행자들
        self.object_publisher = None
        self.ego_publisher = None

    def connect(self):
        with self.lock:
            if self.client:
                try:
                    self.client.terminate()
                except:
                    pass
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.on_ready(self.on_ready, run_in_thread=True)
            self.client.on('close', self.on_close)
            self.client.on('error', self.on_error)

    def on_ready(self):
        rospy.loginfo(f'Connected to rosbridge at {self.host}:{self.port}')
        with self.lock:
            self.connected = True
            # roslibpy 발행자들 초기화
            self.object_publisher = roslibpy.Topic(self.client, '/delivery_object', 'morai_msgs/ObjectStatusList')
            self.intarray_publisher = roslibpy.Topic(self.client, self.intarray_topic, self.intarray_type)
            
            # ROS 구독자들 초기화
            self.object_subscriber = rospy.Subscriber('/Object_topic', ObjectStatusList, self.object_callback)
            self.ego_subscriber = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)

    def on_close(self):
        rospy.logwarn('rosbridge 연결이 끊어졌습니다. 재연결 시도 중...')
        with self.lock:
            self.connected = False
            self.object_publisher = None
            self.intarray_publisher = None
        self.reconnect_later()

    def on_error(self, *args):
        rospy.logwarn(f'rosbridge 에러 발생: {args}. 재연결 시도 중...')
        with self.lock:
            self.connected = False
            self.object_publisher = None
            self.intarray_publisher = None
        self.reconnect_later()

    def reconnect_later(self):
        if self.should_run:
            threading.Timer(RECONNECT_INTERVAL, self.connect).start()

    def object_callback(self, msg):
        if not self.connected or not self.object_publisher:
            return
            
        self.obj_msg_.header = msg.header
        self.obj_msg_.pedestrian_list.clear()
        self.obj_msg_.obstacle_list.clear()

        for ped in msg.pedestrian_list:
            if ped.unique_id == 50:
                self.ped_data_ = ped
        for obs in msg.obstacle_list:
            if obs.unique_id == 51:
                self.obj_1_data_ = obs
            if obs.unique_id == 52:
                self.obj_2_data_ = obs

        self.obj_msg_.pedestrian_list.append(self.ped_data_)
        self.obj_msg_.obstacle_list.append(self.obj_1_data_)
        self.obj_msg_.obstacle_list.append(self.obj_2_data_)
        
        try:
            object_data = {
                'header': {
                    'stamp': {
                        'secs': self.obj_msg_.header.stamp.secs,
                        'nsecs': self.obj_msg_.header.stamp.nsecs
                    },
                    'frame_id': self.obj_msg_.header.frame_id
                },
                'num_of_pedestrian': self.obj_msg_.num_of_pedestrian,
                'num_of_obstacle': self.obj_msg_.num_of_obstacle,
                'pedestrian_list': [],
                'obstacle_list': []
            }
            
            for ped in self.obj_msg_.pedestrian_list:
                ped_data = {
                    'unique_id': ped.unique_id,
                    'name': ped.name,
                    'type': ped.type,
                    'position': {
                        'x': ped.position.x,
                        'y': ped.position.y,
                        'z': ped.position.z
                    },
                    'velocity': {
                        'x': ped.velocity.x,
                        'y': ped.velocity.y,
                        'z': ped.velocity.z
                    },
                    'heading': ped.heading,
                    'size': {
                        'x': ped.size.x,
                        'y': ped.size.y,
                        'z': ped.size.z
                    }
                }
                object_data['pedestrian_list'].append(ped_data)
            
            for obs in self.obj_msg_.obstacle_list:
                obs_data = {
                    'unique_id': obs.unique_id,
                    'name': obs.name,
                    'type': obs.type,
                    'position': {
                        'x': obs.position.x,
                        'y': obs.position.y,
                        'z': obs.position.z
                    },
                    'velocity': {
                        'x': obs.velocity.x,
                        'y': obs.velocity.y,
                        'z': obs.velocity.z
                    },
                    'heading': obs.heading,
                    'size': {
                        'x': obs.size.x,
                        'y': obs.size.y,
                        'z': obs.size.z
                    }
                }
                object_data['obstacle_list'].append(obs_data)
            
            self.object_publisher.publish(object_data)
        except Exception as e:
            rospy.logwarn(f'Object_topic publish 실패: {e}')

    def ego_callback(self, msg):
        if not self.connected:
            return
            
        self.last_ego_data = msg
        _ego_pos = np.array([msg.position.x, msg.position.y])
        _ped_pos = np.array([self.ped_data_.position.x, self.ped_data_.position.y])
        _obj_1_pos = np.array([self.obj_1_data_.position.x, self.obj_1_data_.position.y])
        _obj_2_pos = np.array([self.obj_2_data_.position.x, self.obj_2_data_.position.y])

        _ped_dis = np.linalg.norm(_ego_pos - _ped_pos)
        _obj_1_dis = np.linalg.norm(_ego_pos - _obj_1_pos)
        _obj_2_dis = np.linalg.norm(_ego_pos - _obj_2_pos)
        
        if _obj_1_dis < self.max_distance_:
            if (rospy.Time.now() - self.goal_free_time_).to_sec() > 2.0:
                self.obj_reach_.data[1] = 1
        elif _obj_2_dis < self.max_distance_:
            if (rospy.Time.now() - self.goal_free_time_).to_sec() > 2.0:
                self.obj_reach_.data[2] = 1
        elif _ped_dis < self.max_distance_:
            if (rospy.Time.now() - self.goal_free_time_).to_sec() > 2.0 and not self.reach_flag_:
                self.obj_reach_.data[0] += 1
                self.reach_flag_ = True
        else:
            self.goal_free_time_ = rospy.Time.now()
            self.reach_flag_ = False
        
        try:
            self.intarray_publisher.publish({'data': self.obj_reach_.data})
        except Exception as e:
            rospy.logwarn(f'delivery_check publish 실패: {e}')

    def shutdown(self):
        self.should_run = False
        with self.lock:
            if self.object_subscriber:
                self.object_subscriber.unregister()
            if self.ego_subscriber:
                self.ego_subscriber.unregister()
            if self.client:
                try:
                    self.client.terminate()
                except:
                    pass

def sigint_handler(signum, frame):
    rospy.loginfo("Ctrl+C 감지, 안전하게 종료합니다.")
    processor.shutdown()
    os._exit(0)

if __name__ == "__main__":
    rospy.loginfo("------------------ Object Topic Processor 켜짐 ------------------")
    rospy.init_node('object_topic_processor', anonymous=True)
    processor = ObjectTopicProcessor('127.0.0.1', 9090)
    processor.connect()
    signal.signal(signal.SIGINT, sigint_handler)
    
    try:
        processor.client.run_forever()
    except Exception as e:
        rospy.logwarn(f"예외 발생: {e}")
    finally:
        processor.shutdown()
            
