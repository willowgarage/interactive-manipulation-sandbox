class TopicInfo:
    '''
    Contains information about a single topic.

    This topic may exist on the remote ROS system, the local
    ROS system, or both.
    '''
    def __init__(self, local_topic, message_type=None, message_class=None, md5sum=None):
        self.update(local_topic, message_type, message_class, md5sum)
        self.subscribers = []
        self.publishers = []
        self.publisher = None
        self.subscriber = None

    def update(self, local_topic, message_type=None, message_class=None, md5sum=None):
        self.local_topic = local_topic
        self.message_type = message_type
        self.md5sum = md5sum
        self.message_class = message_class
