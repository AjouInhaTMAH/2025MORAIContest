from bridge.rosmsg_util import rosmsg_to_dict
from processors.processor_base import BaseProcessor, processor_decorator
 
class SensorsCoreProcessor(BaseProcessor):
    @processor_decorator
    def process(self, msg, publisher, output_topic):
        data = rosmsg_to_dict(msg)
        return data 