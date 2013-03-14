class SubscribedTopic:
    def __init__(self):
        self.topic = None
        self.rate = None
        self.compression = None
        self.bytes_sent = 0
        self.bytes_received = 0
        self.last_forwarded_t = None
