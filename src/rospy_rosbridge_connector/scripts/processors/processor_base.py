from abc import ABC, abstractmethod

def processor_decorator(func):
    def wrapper(self, msg, publisher, output_topic):
        processed = func(self, msg, publisher, output_topic)
        if processed is not None:
            publisher.publish(output_topic, processed)
    return wrapper

class BaseProcessor(ABC):
    def __init__(self):
        super().__init__()

    @abstractmethod
    def process(self, msg, publisher, output_topic):
        pass 