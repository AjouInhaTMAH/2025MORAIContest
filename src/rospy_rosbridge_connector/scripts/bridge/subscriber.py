import rospy
from importlib import import_module

class RosSubscriber:
    def __init__(self, topic_specs, callback_map):
        self.subs = []
        for spec in topic_specs:
            msg_module, msg_type = spec['type'].split('/')
            msg_class = getattr(import_module(f"{msg_module}.msg"), msg_type)
            cb = callback_map[spec['processor']]
            sub = rospy.Subscriber(spec['input'], msg_class, cb)
            self.subs.append(sub) 