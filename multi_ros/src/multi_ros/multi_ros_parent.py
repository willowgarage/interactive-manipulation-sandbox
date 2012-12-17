import threading
import cPickle as pickle
import zmq
import rospy, roslib, roslib.message

from multi_ros.ros_interface import RosInterface

class MultiRosParent:
    def __init__(self, ros_master_uri, config_uri, pub_uri, sub_uri, config_dict, prefix='', poll_rate=1.):
        self._ros_master_uri = ros_master_uri
        self._config_uri = config_uri
        self._pub_uri = pub_uri
        self._sub_uri = sub_uri
        self._config_dict = config_dict
        self._prefix = config_dict['prefix']
        self._poll_rate = poll_rate

    def initialize(self):
        # socket for configuration requests
        self._zmq_context = zmq.Context()
        self._zmq_config_socket = self._zmq_context.socket(zmq.REQ)
        self._zmq_config_socket.connect(self._config_uri)

        # socket for receiving messages
        rospy.loginfo('Parent listening for messages on zmq socket %s' % self._pub_uri)
        self._zmq_pub_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_pub_socket.connect(self._pub_uri)
        self._zmq_pub_socket.setsockopt(zmq.SUBSCRIBE, '')
        self._pub_socket_lock = threading.Lock()

        # socket for publishing messages
        rospy.loginfo('Parent publishing messages on zmq socket %s' % self._sub_uri)
        self._zmq_sub_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_sub_socket.connect(self._sub_uri)
        self._sub_socket_lock = threading.Lock()

        # used to inspect the local ROS system
        self._ros_interface = RosInterface(self._ros_master_uri, 'mros_parent', self.ros_message_callback)
        self._ros_interface_lock = threading.Lock()

        # start thread that publishes ROS messages we get from remote system
        #publish_thread = threading.Thread(target=self.publish_thread_func)
        #publish_thread.start()

    def parent_to_child_topic(self, topic):
        return topic[len(self._prefix):]

    def child_to_parent_topic(self, topic):
        return self._prefix + topic

    def run(self):
        self.initialize()

        child_topics = []
        print self._config_dict
        for child_topic in self._config_dict['topics']:
            parent_topic = self.child_to_parent_topic(child_topic)
            
            topic_dict = self._config_dict['topics'][child_topic]
            message_type = str(topic_dict['message_type'])
            message_class = roslib.message.get_message_class(message_type)
            md5sum = str(message_class._md5sum)
            child_topics.append((child_topic, message_type, md5sum))

            # subscribe to the topic on the local ROS system
            rospy.loginfo('Parent subscribing to %s' % parent_topic)
            self._ros_interface.subscribe(parent_topic, message_type, md5sum)

            # advertise the topic on the local ROS system
            rospy.loginfo('Parent advertising %s' % parent_topic)
            self._ros_interface.advertise(parent_topic, message_type, md5sum)

        # tell the child to advertise each of the forwarded topics
        command = ['ADVERTISE'] + child_topics
        self._zmq_config_socket.send(pickle.dumps(command))
        rep = pickle.loads(self._zmq_config_socket.recv())

        # tell the child to subscribe to each of the forwarded topics
        command = ['SUBSCRIBE'] + child_topics
        self._zmq_config_socket.send(pickle.dumps(command))
        rep = pickle.loads(self._zmq_config_socket.recv())

        while not rospy.is_shutdown():
            rospy.logdebug('Parent waiting for messages from child')
            child_topic, msg = pickle.loads(self._zmq_pub_socket.recv())
            parent_topic = self.child_to_parent_topic(child_topic)
            rospy.logdebug('Parent publishing message from child topic %s on topic %s' % (child_topic, parent_topic))
            with self._ros_interface_lock:
                self._ros_interface.publish(parent_topic, msg)

    def run_dynamic(self):
        self.initialize()
        
        while not rospy.is_shutdown():
            rospy.loginfo('MRos parent updating')
            
            # get a list of topics from the remote
            self._zmq_config_socket.send(pickle.dumps(('GET_TOPIC_LIST',)))
            remote_ros_graph = pickle.loads(self._zmq_config_socket.recv())

            # inspect the local ros graph
            self._ros_interface.update_ros_graph()
            local_ros_graph = self._ros_interface.get_ros_graph()

            for topic in remote_ros_graph:
                # all topics that exist on the remote system should have local publishers, so that
                # they are visible on the local ROS master
                rospy.loginfo('Parent advertising %s' % topic)
                self._ros_interface.advertise(topic, remote_ros_graph[topic]['message_type'], remote_ros_graph[topic]['md5sum'])

                # if there is a subscriber on the remote side, there needs to be a subscriber
                # on the local side, so that we can forward along all messages on this topic
                # TODO: destroy local subscriber when there are no more remote subscribers
                if len(remote_ros_graph[topic]['subscribers']) > 0:
                    rospy.loginfo('Parent subscribing to %s' % topic)
                    self._ros_interface.subscribe(topic, remote_ros_graph[topic]['message_type'], remote_ros_graph[topic]['md5sum'])

            topics_for_subscription = []
            topics_for_advertising = []
            for topic in local_ros_graph:
                # all topics which are subscribed to locally need to be subscribed to remotely
                print topic, local_ros_graph[topic]['subscribers']
                if len(local_ros_graph[topic]['subscribers']) > 0:
                    # FIXME just for testing
                    if not topic in ['/test_topic']:
                        continue
                    
                    topics_for_subscription.append((topic, local_ros_graph[topic]['message_type'], local_ros_graph[topic]['md5sum']))

                # all topics which are published locally need to be advertised remotely
                topics_for_advertising.append((topic, local_ros_graph[topic]['message_type'], local_ros_graph[topic]['md5sum']))

            # send list of topics that the remote side should subscribe to
            command = ['SUBSCRIBE'] + topics_for_subscription
            self._zmq_config_socket.send(pickle.dumps(command))
            rep = pickle.loads(self._zmq_config_socket.recv())

            # send list of topics that the remote side should advertise
            command = ['ADVERTISE'] + topics_for_advertising
            self._zmq_config_socket.send(pickle.dumps(command))
            rep = pickle.loads(self._zmq_config_socket.recv())
            
        rospy.spin()
        
    def publish_thread_func(self):
        while not rospy.is_shutdown():
            topic, msg = pickle.loads(self._zmq_pub_socket.recv())
            rospy.loginfo('Parent publishing message on %s' % topic)            
            with self._ros_interface_lock:
                self._ros_interface.publish(topic, msg)

    def ros_message_callback(self, msg, parent_topic):
        # TODO: throttle based on priority
        child_topic = self.parent_to_child_topic(parent_topic)
        with self._sub_socket_lock:
            rospy.logdebug('Parent sending ROS message from topic %s to child for topic %s' % (parent_topic, child_topic))
            self._zmq_sub_socket.send(pickle.dumps((child_topic, msg._buff)))
