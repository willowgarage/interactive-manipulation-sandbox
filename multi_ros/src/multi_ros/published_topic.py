class PublishedTopic(object):
    def __init__(self, topic=None, message_type=None, md5sum=None, compression=None):
        self.topic = topic
        self.compression = compression
        self.message_type = message_type
        self.md5sum = md5sum
        self.bytes_sent = 0
        self.bytes_received = 0
