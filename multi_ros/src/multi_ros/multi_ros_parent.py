import threading
import zmq
import roslib
import roslib.message
import rospy
import cPickle as pickle

from multi_ros.multi_ros_node import MultiRosNode
from multi_ros.published_topic import PublishedTopic
from multi_ros.subscribed_topic import SubscribedTopic


class MultiRosParent(MultiRosNode):
    def __init__(self, name, prefix, ros_master_uri, poll_rate=1.0):
        super(MultiRosParent, self).__init__(name, prefix, ros_master_uri, poll_rate)

    def run_as_parent(self, config_uri, pub_uri, sub_uri, config_dict):
        '''
        Connect to these addressses.
        '''
        # socket for configuration requests
        self._zmq_context = zmq.Context()

        # socket for publishing messages
        self._zmq_pub_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_pub_lock = threading.Lock()

        # socket for receiving messages
        self._zmq_sub_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_sub_socket.setsockopt(zmq.SUBSCRIBE, '')

        rospy.loginfo('%s connecting to %s forconf' % (self._name, config_uri))
        self._zmq_config_socket = self._zmq_context.socket(zmq.REQ)
        self._zmq_config_socket.connect(config_uri)
        rospy.loginfo('%s connecting to %s for pub' % (self._name, pub_uri))
        self._zmq_pub_socket.connect(pub_uri)
        rospy.loginfo('%s connecting to %s for sub' % (self._name, sub_uri))
        self._zmq_sub_socket.connect(sub_uri)

        remote_sub_topics = []
        remote_pub_topics = []
        rospy.loginfo('%s connecting: %s' % (self._name, str(config_dict)))
        for topic_dict in config_dict['topics']:
            remote_topic = str(topic_dict['topic'])
            local_topic = self.remote_to_local_topic(remote_topic)

            message_type = str(topic_dict['message_type'])
            message_class = roslib.message.get_message_class(message_type)
            if 'compression' in topic_dict:
                compression = topic_dict['compression']
            else:
                compression = None
            if 'rate' in topic_dict:
                rate = topic_dict['rate']
            else:
                rate = None
            md5sum = str(message_class._md5sum)

            remote_sub_topics.append({
                'topic': remote_topic, 'message_type': message_type,
                'md5sum': md5sum, 'compression': compression, 'rate': rate})
            remote_pub_topics.append({
                'topic': remote_topic, 'message_type': message_type,
                'md5sum': md5sum, 'compression': compression})

            sub_topic_info = SubscribedTopic()
            sub_topic_info.topic = local_topic
            sub_topic_info.message_type = message_type
            sub_topic_info.md5sum = md5sum
            sub_topic_info.rate = rate
            sub_topic_info.compression = compression
            self._sub_topics[sub_topic_info.topic] = sub_topic_info

            pub_topic_info = PublishedTopic()
            pub_topic_info.topic = local_topic
            pub_topic_info.message_type = message_type
            pub_topic_info.md5sum = md5sum
            pub_topic_info.compression = compression
            self._pub_topics[pub_topic_info.topic] = pub_topic_info

            # subscribe to the topic on the local ROS system
            rospy.loginfo('%s subscribing to %s' % (self._name, local_topic))
            self._ros_interface.subscribe(local_topic, message_type, md5sum)

            # advertise the topic on the local ROS system
            rospy.loginfo('%s advertising %s' % (self._name, local_topic))
            self._ros_interface.advertise(local_topic, message_type, md5sum)

        # tell the remote to advertise each of the forwarded topics
        command = {'COMMAND': 'ADVERTISE', 'TOPICS': remote_pub_topics}
        self._zmq_config_socket.send(pickle.dumps(command))
        pickle.loads(self._zmq_config_socket.recv())

        # tell the remote to subscribe to each of the forwarded topics
        command = {'COMMAND': 'SUBSCRIBE', 'TOPICS': remote_sub_topics}
        self._zmq_config_socket.send(pickle.dumps(command))
        pickle.loads(self._zmq_config_socket.recv())

        # spin an wait for remote messages to publish
        self.remote_message_thread_func()
