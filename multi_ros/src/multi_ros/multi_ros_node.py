import threading
import zlib
import time
import cPickle as pickle
import rospy
import zmq
from multi_ros.ros_interface import RosInterface


class MultiRosNode(object):
    def __init__(self, name='MultiRosNode', ros_master_uri=None, poll_rate=1.0):
        self._name = name
        self._ros_master_uri = ros_master_uri

        self._poll_rate = poll_rate

        # These dictionaries must be populated by the child classes to forward messages
        self._pub_topics = {}
        self._pub_topics_lock = threading.Lock()
        self._sub_topics = {}
        self._sub_topics_lock = threading.Lock()

        # used to interact the local ROS system
        self._ros_interface = RosInterface(self._ros_master_uri, self._name, self.local_message_callback)

        # socket for configuration requests
        self._zmq_context = zmq.Context()

        # socket for publishing messages
        self._zmq_pub_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_pub_lock = threading.Lock()

        # socket for receiving messages
        self._zmq_sub_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_sub_socket.setsockopt(zmq.SUBSCRIBE, '')
        self._shutdown = False

    def __del__(self):
        self._shutdown = True

    def local_message_callback(self, msg, topic):
        '''
        Receive a message on the local ROS system. Forward it to the remote system.
        '''
        with self._sub_topics_lock:
            if topic not in self._sub_topics:
                rospy.logwarn('%s: Message received on %s but no config file rule for this topic' % (self._name, topic))
                return
            topic_info = self._sub_topics[topic]

        current_t = time.time()
        if topic_info.rate is not None:
            if topic_info.last_forwarded_t is not None:
                if (current_t - topic_info.last_forwarded_t) < (1.0/topic_info.rate):
                    #  just forwarded a message on this topic ignore this one
                    return
        topic_info.last_forwarded_t = current_t
        self.forward_message(msg, topic_info, topic)

    def forward_message(self, msg, topic_info,  topic):
        """
        Forwards a message to the topic of the same name on the remote system
        """
        if self._zmq_pub_socket is not None:
            with self._zmq_pub_lock:
                rospy.loginfo('%s forwarding message to topic %s on the remote system' % (self._name, topic))
                if topic_info.compression is None:
                    msg_buf = msg._buff
                    rospy.logdebug('Topic: %s  Uncompressed: %d' % (topic, len(msg._buff), len(msg_buf)))
                elif topic_info.compression == 'zlib':
                    msg_buf = zlib.compress(msg._buff)
                    rospy.logdebug('Topic: %s  Uncompressed: %d  Compressed: %d' % (topic, len(msg._buff), len(msg_buf)))
                else:
                    rospy.logerr('Unknown compression type %s for topic %s' % (str(topic_info.compression), topic))
                    return

                self._zmq_pub_socket.send(pickle.dumps((topic, msg_buf)))

    def remote_forwarding_loop(self):
        '''
        Receive messages from the remote ROS system and republish them locally.
        '''
        rospy.loginfo('%s: starting message listener' % self._name)
        while not rospy.is_shutdown() and not self._shutdown:
            msg_str = self._zmq_sub_socket.recv()
            print '%s: message received' % self._name
            remote_topic, msg_buf = pickle.loads(msg_str)
            local_topic = self.remote_to_local_topic(remote_topic)
            with self._pub_topics_lock:
                if not local_topic in self._pub_topics:
                    rospy.logwarn('%s received message on unconfigured topic %s' % (self._name, local_topic))
                    continue
                topic_info = self._pub_topics[local_topic]

            if topic_info.compression == 'zlib':
                msg_buf = zlib.decompress(msg_buf)
            elif topic_info.compression is not None:
                rospy.logerr('Unknown compression type %s for topic %s' % (str(topic_info.compression), local_topic))
                continue

            rospy.loginfo('%s: publishing received message to topic %s' % (self._name, local_topic))
            self._ros_interface.publish(local_topic, msg_buf)

    def remote_to_local_topic(self, remote_topic):
        """
        Default conversion assumes remote and local are the same
        """
        return remote_topic
