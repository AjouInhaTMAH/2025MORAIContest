import roslibpy

class RoslibpyPublisher:
    def __init__(self, remote_ip, port, topic_specs):
        self.ros = roslibpy.Ros(host=remote_ip, port=port)
        self.ros.run()
        self.publishers = {}
        for spec in topic_specs:
            self.publishers[spec['output']] = roslibpy.Topic(self.ros, spec['output'], spec['type'])

    def publish(self, topic, msg):
        self.publishers[topic].publish(roslibpy.Message(msg)) 