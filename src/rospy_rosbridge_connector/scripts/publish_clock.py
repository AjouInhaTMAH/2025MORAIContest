#!/usr/bin/env python3
import time
import rospy
from rosgraph_msgs.msg import Clock
import roslibpy
import sys
import os
import logging
import threading
import signal

# Twisted/roslibpy 로그 최소화
logging.getLogger('roslibpy').setLevel(logging.CRITICAL)
logging.getLogger('twisted').setLevel(logging.CRITICAL)

RECONNECT_INTERVAL = 1  # 재연결 시도 간격 (초)

class RosbridgeClockPublisher:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client = None
        self.publisher = None
        self.connected = False
        self.should_run = True
        self.lock = threading.Lock()

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
        # run_forever는 메인스레드에서 실행해야 하므로 여기서 호출하지 않음

    def on_ready(self):
        rospy.loginfo(f'Connected to rosbridge at {self.host}:{self.port}')
        with self.lock:
            self.connected = True
            self.publisher = roslibpy.Topic(self.client, '/clock', 'rosgraph_msgs/Clock')

    def on_close(self):
        rospy.logwarn('rosbridge 연결이 끊어졌습니다. 재연결 시도 중...')
        with self.lock:
            self.connected = False
            self.publisher = None
        self.reconnect_later()

    def on_error(self, *args):
        rospy.logwarn(f'rosbridge 에러 발생: {args}. 재연결 시도 중...')
        with self.lock:
            self.connected = False
            self.publisher = None
        self.reconnect_later()

    def reconnect_later(self):
        if self.should_run:
            threading.Timer(RECONNECT_INTERVAL, self.connect).start()

    def publish_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.should_run:
            with self.lock:
                if self.connected and self.publisher:
                    now = rospy.Time.now()
                    clock_msg = {
                        'clock': {
                            'secs': now.secs,
                            'nsecs': now.nsecs
                        }
                    }
                    try:
                        self.publisher.publish(clock_msg)
                    except Exception as e:
                        rospy.logwarn(f'publish 실패: {e}')
            rate.sleep()

    def shutdown(self):
        self.should_run = False
        with self.lock:
            if self.publisher:
                try:
                    self.publisher.unadvertise()
                except:
                    pass
            if self.client:
                try:
                    self.client.terminate()
                except:
                    pass

def sigint_handler(signum, frame):
    rospy.loginfo("Ctrl+C 감지, 안전하게 종료합니다.")
    publisher.shutdown()
    os._exit(0)

if __name__ == "__main__":
    rospy.loginfo("------------------ publish clock 켜짐 ------------------")
    rospy.init_node('morai_clock_publisher', anonymous=True)
    publisher = RosbridgeClockPublisher('127.0.0.1', 9090)
    publisher.connect()
    pub_thread = threading.Thread(target=publisher.publish_loop, daemon=True)
    pub_thread.start()
    signal.signal(signal.SIGINT, sigint_handler)  # Ctrl+C 핸들러 등록
    try:
        # run_forever는 반드시 메인스레드에서 실행
        publisher.client.run_forever()
    except Exception as e:
        rospy.logwarn(f"예외 발생: {e}")
    finally:
        publisher.shutdown() 