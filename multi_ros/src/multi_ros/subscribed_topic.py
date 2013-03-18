class SubscribedTopic(object):
    def __init__(self, topic=None, message_type=None, md5sum=None, rate=None, compression=None):
        self.topic = topic
        self.rate = rate
        self.message_type = message_type
        self.md5sum = md5sum
        self.compression = compression
        self.bytes_sent = 0
        self.bytes_received = 0
        self.last_forwarded_t = None
