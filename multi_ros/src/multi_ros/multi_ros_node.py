import threading, zlib, time
import json
import zmq
import rospy

from multi_ros.ros_interface import RosInterface

class PublishedTopic:
    def __init__(self, topic):
        self.topic = topic
        self.compression = None
        self.bytes_sent = 0
        self.bytes_received = 0
        
class SubscribedTopic:
    def __init__(self, topic):
        self.topic = topic
        self.rate = None
        self.compression = None
        self.bytes_sent = 0
        self.bytes_received = 0

class MultiRosNode:
    def __init__(self, name, prefix, ros_master_uri, config_uri, pub_uri, sub_uri, poll_rate=1.0):
        self._name = name
        self._prefix = prefix
        self._ros_master_uri = ros_master_uri
        self._config_uri = config_uri
        self._pub_uri = pub_uri
        self._sub_uri = sub_uri
        self._poll_rate = poll_rate        
        self._pub_topics = {}
        self._pub_topics_lock = threading.Lock()
        self._sub_topics = {}
        self._sub_topics_lock = threading.Lock()

    def initialize(self):
        '''
        Create ZMQ sockets and initialize ROS client.

        Call this before calling connect(), bind(), or run().
        '''
        # socket for configuration requests
        self._zmq_context = zmq.Context()
        self._zmq_config_socket = self._zmq_context.socket(zmq.REP)
        self._zmq_config_socket.bind(self._config_uri)

        # socket for publishing messages
        rospy.loginfo('%s publishing messages on zmq socket %s' % (self._name, self._pub_uri))
        self._zmq_pub_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_pub_lock = threading.Lock()

        # socket for receiving messages
        rospy.loginfo('%s listening for messages on zmq socket %s' % (self._name, self._sub_uri))
        self._zmq_sub_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_sub_socket.setsockopt(zmq.SUBSCRIBE, '')

        # used to interact the local ROS system
        self._ros_interface = RosInterface(self._ros_master_uri, self._name, self.local_message_callback)

        self._remote_message_thread = threading.Thread(target=self.remote_message_thread_func)
        self._remote_message_thread.start()

    def bind(self, pub_uri, sub_uri):
        '''
        Bind to these addresses.
        '''
        rospy.loginfo('%s binding to %s for publishing' % (self._name, pub_uri))
        self._zmq_pub_socket.bind(pub_uri)
        rospy.loginfo('%s binding to %s for subscribing' % (self._name, pub_uri))
        self._zmq_sub_socket.bind(sub_uri)

    def connect(self, pub_uri, sub_uri):
        '''
        Connect to these addressses.
        '''
        rospy.loginfo('%s connecting to %s for publishing' % (self._name, pub_uri))
        self._zmq_pub_socket.connect(pub_uri)
        rospy.loginfo('%s connecting to %s for subscribing' % (self._name, pub_uri))
        self._zmq_sub_socket.connect(sub_uri)

    def run(self):
        loop_rate = rospy.Rate(self._poll_rate)
        while not rospy.is_shutdown():
            msg_str = self._zmq_config_socket.recv()
            req = json.loads(msg_str)
            cmd = req['COMMAND']
            if cmd == 'GET_TOPIC_LIST':
                self._ros_interface.update_ros_graph()
                rospy.loginfo('Child sending topic list')
                local_topics = self._ros_interface.get_ros_graph()
                self._zmq_config_socket.send(json.dumps(local_topics))
            elif cmd == 'ADVERTISE':
                for topic_dict in req['topics']:
                    topic_info = PublishedTopic()
                    topic_info.topic = topic_dict['topic']
                    topic_info.message_type = topic_dict['message_type']
                    topic_info.md5sum = topic_dict['md5sum']
                    topic_info.compression = topic_dict.get('compression', None)
                    rospy.loginfo('%s advertising %s' % (self._name, topic_info.topic))
                    self._ros_interface.advertise(topic_info.topic, topic_info.message_type, topic_info.md5sum)
                    with self._pub_topics_lock:
                        self._pub_topics[topic] = topic_info
                self._zmq_config_socket.send(json.dumps('OK'))
            elif cmd == 'SUBSCRIBE':
                for topic_dict in req['topics']:
                    topic_info = AdvertisedTopic()
                    topic_info.topic = topic_dict['topic']
                    topic_info.message_type = topic_dict['message_type']
                    topic_info.md5sum = topic_dict['md5sum']
                    topic_info.compression = topic_dict.get('compression', None)
                    topic_info.rate = topic_dict.get('rate', None)
                    rospy.loginfo('%s subscribing to %s' % (self._name, topic_info.topic))
                    self._ros_interface.subscribe(topic_info.topic, topic_info.message_type, topic_info.md5sum)
                    with self._pub_topics_lock:
                        self._pub_topics[topic] = topic_info
                self._zmq_config_socket.send(json.dumps('OK'))
            else:
                rospy.logerr('Unknown command %s' % cmd)
                self._zmq_config_socket.send(json.dumps('ERROR'))
            
            loop_rate.sleep()

    def local_message_callback(self, msg, topic):
        '''
        Received a message on the local ROS system. Forward it to the remote system.
        '''
        with self._sub_topics_lock:
            if not topic in self._sub_topics:
                rospy.logwarn('Message received on %s but no config info for this topic' % topic)
                return
            topic_info = self._sub_topics[topic]

        current_t = time.time()
        if topic_info.rate is not None:
            if topic_info.last_forwarded_t is not None:
                if (current_t - topic_info.last_forwarded_t)  < (1.0/topic_info.rate):
                    # just forwarded a message on this topic; don't need to forward this one
                    return
        topic_info.last_forwarded_t = current_t
            
        if self._zmq_pub_socket is not None:
            with self._zmq_pub_lock:
                rospy.logdebug('Child forwarding message from %s' % topic)
                if topic_info.compression is None:
                    msg_buf = msg._buf
                elif topic_info.compression == 'zlib':
                    msg_buf = zlib.compress(msg._buff)
                else:
                    rospy.logerr('Unknown compression type %s for topic %s' % (str(topic_info.compression), topic))
                    return
                
                rospy.loginfo('Topic: %s  Uncompressed: %d  Compressed: %d' % (topic, len(msg._buff), len(msg_buf)))
                self._zmq_pub_socket.send(json.dumps((topic, msg_buf)))

    def remote_message_thread_func(self):
        '''
        Receive messages from the remote ROS system and republish
        them locally.
        '''
        while not rospy.is_shutdown():
            msg_str = self._zmq_sub_socket.recv()
            topic, msg_buf = json.loads(msg_str)
            with self._pub_topics_lock:
                if not topic in self._pub_topics:
                    rospy.logwarn('%s received message on unconfigured topic %s' % (self._name, topic))
                    continue
                topic_info = self._pub_topics[topic]

            if topic_info.compression is None:
                msg_buf = msg._buf
            elif topic_info.compression == 'zlib':
                msg_buf = zlib.decompress(msg_buf)
            else:
                rospy.logerr('Unknown compression type %s for topic %s' % (str(topic_info.compression), topic))
                continue

            
            rospy.logdebug('%s publishing message received over link to topic %s' % (self._name, topic))
            self._ros_interface.publish(topic, msg_buf)

    def remote_to_local_topic(self, remote_topic):
        return remote_topic[len(self._prefix):]

    def local_to_remote_topic(self, local_topic):
        return self._prefix + local_topic
