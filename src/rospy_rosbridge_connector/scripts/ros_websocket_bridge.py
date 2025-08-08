#!/usr/bin/env python3
import yaml
import rospy
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)
from bridge.subscriber import RosSubscriber
from bridge.publisher import RoslibpyPublisher
import importlib
import os
import rospkg
import time
import roslibpy
import sys
import logging
import threading
import signal

# Twisted/roslibpy 로그 최소화
logging.getLogger('roslibpy').setLevel(logging.CRITICAL)
logging.getLogger('twisted').setLevel(logging.CRITICAL)

RECONNECT_INTERVAL = 1  # 재연결 시도 간격 (초)

def load_topic_specs():
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('rospy_rosbridge_connector')
    config_path = os.path.join(base_path, 'config', 'topic_specs.yaml')
    with open(config_path) as f:
        return yaml.safe_load(f)['topics']

def get_callback_map(topic_specs, publisher):
    callback_map = {}
    for spec in topic_specs:
        module = importlib.import_module(f"processors.{spec['processor']}")
        processor_name = spec['processor']
        if processor_name == 'GetTrafficLightStatus':
            class_name = 'GetTrafficLightStatusProcessor'
        elif processor_name == 'Object_topic':
            class_name = 'ObjectTopicProcessor'
        elif processor_name == 'servo_position_command':
            class_name = 'ServoPositionCommandProcessor'
        elif processor_name == 'sensors_core':
            class_name = 'SensorsCoreProcessor'
        else:
            class_name = ''.join(word.capitalize() for word in processor_name.split('_')) + 'Processor'
        processor_class = getattr(module, class_name)
        processor_instance = processor_class()
        callback_map[spec['processor']] = lambda msg, p=processor_instance, out=spec['output']: p.process(msg, publisher, out)
    return callback_map

class RosbridgeBridgeManager:
    def __init__(self, remote_ip, port, topic_specs):
        self.remote_ip = remote_ip
        self.port = port
        self.topic_specs = topic_specs
        self.client = None
        self.publisher = None
        self.callback_map = None
        self.sub = None
        self.lock = threading.Lock()
        self.should_run = True

    def connect(self):
        with self.lock:
            if self.client:
                try:
                    self.client.terminate()
                except:
                    pass
            self.client = roslibpy.Ros(host=self.remote_ip, port=self.port)
            self.client.on_ready(self.on_ready, run_in_thread=True)
            self.client.on('close', self.on_close)
            self.client.on('error', self.on_error)
        # run_forever는 메인스레드에서 실행해야 하므로 여기서 호출하지 않음

    def on_ready(self):
        rospy.loginfo(f'Connected to rosbridge at {self.remote_ip}:{self.port}')
        with self.lock:
            self.publisher = RoslibpyPublisher(self.remote_ip, self.port, self.topic_specs)
            self.publisher.ros = self.client
            self.callback_map = get_callback_map(self.topic_specs, self.publisher)
            if self.sub is None:
                self.sub = RosSubscriber(self.topic_specs, self.callback_map)

    def on_close(self):
        rospy.logwarn('rosbridge 연결이 끊어졌습니다. 재연결 시도 중...')
        with self.lock:
            self.publisher = None
            self.callback_map = None
        self.reconnect_later()

    def on_error(self, *args):
        rospy.logwarn(f'rosbridge 에러 발생: {args}. 재연결 시도 중...')
        with self.lock:
            self.publisher = None
            self.callback_map = None
        self.reconnect_later()

    def reconnect_later(self):
        if self.should_run:
            threading.Timer(RECONNECT_INTERVAL, self.connect).start()

    def shutdown(self):
        self.should_run = False
        with self.lock:
            if self.client:
                try:
                    self.client.terminate()
                except:
                    pass

if __name__ == '__main__':
    rospy.loginfo("------------------ ros websocket bridge 켜짐 ------------------")
    rospy.init_node('ros_websocket_bridge_mo')
    remote_ip = rospy.get_param('~remote_ip', '127.0.0.1')
    port = int(rospy.get_param('~port', 9090))
    topic_specs = load_topic_specs()
    manager = RosbridgeBridgeManager(remote_ip, port, topic_specs)
    manager.connect()

    def sigint_handler(signum, frame):
        rospy.loginfo("Ctrl+C 감지, 안전하게 종료합니다.")
        manager.shutdown()
        os._exit(0)
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        manager.client.run_forever()
    except Exception as e:
        rospy.logwarn(f"예외 발생: {e}")
    finally:
        manager.shutdown()
