import threading, zlib, time
import cPickle as pickle
import zmq
import rospy

from multi_ros.ros_interface import RosInterface

class MultiRosChild:
    def __init__(self, ros_master_uri, config_uri, pub_uri, sub_uri, poll_rate=1.0):
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
        # socket for configuration requests
        self._zmq_context = zmq.Context()
        self._zmq_config_socket = self._zmq_context.socket(zmq.REP)
        self._zmq_config_socket.bind(self._config_uri)

        # socket for publishing messages
        rospy.loginfo('Child publishing messages on zmq socket %s' % self._pub_uri)
        self._zmq_pub_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_pub_socket.bind(self._pub_uri)
        self._zmq_pub_lock = threading.Lock()

        # socket for receiving messages
        rospy.loginfo('Child listening for messages on zmq socket %s' % self._sub_uri)
        self._zmq_sub_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_sub_socket.bind(self._sub_uri)
        self._zmq_sub_socket.setsockopt(zmq.SUBSCRIBE, '')

        # used to interact the local ROS system
        self._ros_interface = RosInterface(self._ros_master_uri, 'mros_child', self.local_message_callback)

        self._remote_message_thread = threading.Thread(target=self.remote_message_thread_func)
        self._remote_message_thread.start()

    def run(self):
        self.initialize()
        
        loop_rate = rospy.Rate(self._poll_rate)
        while not rospy.is_shutdown():
            msg_str = self._zmq_config_socket.recv()
            msg = pickle.loads(msg_str)
            cmd = msg[0]
            if cmd == 'GET_TOPIC_LIST':
                self._ros_interface.update_ros_graph()
                rospy.loginfo('Child sending topic list')
                local_topics = self._ros_interface.get_ros_graph()
                self._zmq_config_socket.send(pickle.dumps(local_topics))
            elif cmd == 'ADVERTISE':
                for topic, message_type, message_md5sum, compression in msg[1:]:
                    rospy.loginfo('Child advertising %s' % topic)
                    self._ros_interface.advertise(topic, message_type, message_md5sum)
                    with self._pub_topics_lock:
                        self._pub_topics[topic] = {'compression':compression}
                self._zmq_config_socket.send(pickle.dumps('OK'))
            elif cmd == 'SUBSCRIBE':
                for topic, message_type, message_md5sum, compression, rate in msg[1:]:
                    rospy.loginfo('Child subscribing to %s' % topic)
                    self._ros_interface.subscribe(topic, message_type, message_md5sum)
                    with self._sub_topics_lock:
                        self._sub_topics[topic] = {'compression':compression, 'rate':rate}
                self._zmq_config_socket.send(pickle.dumps('OK'))
            else:
                rospy.logerr('Unknown command %s' % cmd)
                self._zmq_config_socket.send(pickle.dumps('ERROR'))
            
            loop_rate.sleep()

    def local_message_callback(self, msg, topic):
        '''
        Received a message on the local ROS system. Forward it to the remote system.
        '''
        with self._sub_topics_lock:
            if not topic in self._sub_topics:
                rospy.logwarn('Message received on %s but no config info for this topic' % topic)
                return
            compression = self._sub_topics[topic]['compression']
            rate = self._sub_topics[topic]['rate']

        # slight hack. should maybe use the timestamp from the message header instead of
        # the current time?
        current_t = time.time()
        if rate is not None:
            if 'last_forwarded_t' in self._sub_topics[topic]:
                if (current_t - self._sub_topics[topic]['last_forwarded_t'])  < (1.0/rate):
                    # just forwarded a message on this topic; don't need to forward this one
                    return
        self._sub_topics[topic]['last_forwarded_t'] = current_t
            
        if self._zmq_pub_socket is not None:
            with self._zmq_pub_lock:
                rospy.logdebug('Child forwarding message from %s' % topic)
                if compression is None:
                    msg_buf = msg._buf
                elif compression == 'zlib':
                    msg_buf = zlib.compress(msg._buff)
                else:
                    rospy.logerr('Unknown compression type %s for topic %s' % (str(compression), topic))
                    return
                
                rospy.loginfo('Topic: %s  Uncompressed: %d  Compressed: %d' % (topic, len(msg._buff), len(msg_buf)))
                self._zmq_pub_socket.send(pickle.dumps((topic, msg_buf)))

    def remote_message_thread_func(self):
        '''
        Receive messages from the remote ROS system and republish
        them locally.
        '''
        while not rospy.is_shutdown():
            msg_str = self._zmq_sub_socket.recv()
            topic, msg = pickle.loads(msg_str)
            rospy.logdebug('Child publishing message received from parent to %s' % topic)
            self._ros_interface.publish(topic, msg)
