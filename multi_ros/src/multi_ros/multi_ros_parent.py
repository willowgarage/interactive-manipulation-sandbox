import zmq
import roslib
import roslib.message
import rospy
import threading
import cPickle as pickle
import time

from multi_ros.multi_ros_node import MultiRosNode
from multi_ros.published_topic import PublishedTopic
from multi_ros.subscribed_topic import SubscribedTopic


class MultiRosParent(MultiRosNode):
    def __init__(self, config_dict, name='MultiRosParent', ros_master_uri=None, poll_rate=1.0):
        """
        Connects zmq sockets to the child
        """
        super(MultiRosParent, self).__init__(name, ros_master_uri, poll_rate)
        self._prefix = config_dict['prefix']

        config_uri = config_dict['uri'] + ':5000'
        pub_uri = config_dict['uri'] + ':5001'
        sub_uri = config_dict['uri'] + ':5002'
        rospy.loginfo('%s connecting to %s for conf' % (self._name, config_uri))
        self._zmq_config_socket = self._zmq_context.socket(zmq.REQ)
        self._zmq_config_socket.connect(config_uri)
        rospy.loginfo('%s connecting to %s for pub' % (self._name, pub_uri))
        self._zmq_pub_socket.connect(pub_uri)
        rospy.loginfo('%s connecting to %s for sub' % (self._name, sub_uri))
        self._zmq_sub_socket.connect(sub_uri)
        self._config_dict = config_dict
        self.configure_child()
        print 'starting thread'
        self._forwarding_thread = threading.Thread(target=self.remote_forwarding_loop)
        self._forwarding_thread.start()
        print 'thread started'

    def configure_child(self):
        remote_sub_topics = []
        remote_pub_topics = []
        rospy.loginfo('%s connecting: %s' % (self._name, str(self._config_dict)))
        for topic_dict in self._config_dict['topics']:
            remote_topic = str(topic_dict['topic'])
            local_topic = self.remote_to_local_topic(remote_topic)

            message_type = str(topic_dict['message_type'])
            message_class = roslib.message.get_message_class(message_type)
            compression = None
            if 'compression' in topic_dict:
                compression = topic_dict['compression']
            rate = None
            if 'rate' in topic_dict:
                rate = topic_dict['rate']
            md5sum = str(message_class._md5sum)

            remote_sub_topics.append({
                'topic': remote_topic, 'message_type': message_type,
                'md5sum': md5sum, 'compression': compression, 'rate': rate})
            remote_pub_topics.append({
                'topic': remote_topic, 'message_type': message_type,
                'md5sum': md5sum, 'compression': compression})

            self._sub_topics[local_topic] = SubscribedTopic(local_topic, message_type, md5sum, rate, compression)
            self._pub_topics[local_topic] = PublishedTopic(local_topic, message_type, md5sum, compression)

            # subscribe/advertise to the topic on the local ROS system
            rospy.loginfo('%s subscribing to %s' % (self._name, local_topic))
            self._ros_interface.subscribe(local_topic, message_type, md5sum)
            rospy.loginfo('%s advertising %s' % (self._name, local_topic))
            self._ros_interface.advertise(local_topic, message_type, md5sum)

        # tell the remote to advertise/subscribe to each of the forwarded topics
        command = {'COMMAND': 'ADVERTISE', 'TOPICS': remote_pub_topics}
        self._zmq_config_socket.send(pickle.dumps(command))
        pickle.loads(self._zmq_config_socket.recv())

        command = {'COMMAND': 'SUBSCRIBE', 'TOPICS': remote_sub_topics}
        self._zmq_config_socket.send(pickle.dumps(command))
        pickle.loads(self._zmq_config_socket.recv())
        print 'completed configuration'

    # Parent Prefix handling functions
    def forward_message(self, msg, topic_info,  local_topic):
        """
        Overrides the MultiRosNode forward_message to remove the prefix before forwarding the message to the child
        """
        local_topic = local_topic[len(self._prefix):]
        super(MultiRosParent, self).forward_message(msg, topic_info, local_topic)

    def remote_to_local_topic(self, remote_topic):
        """
        Overrides default conversion to add the prefix to the child topic
        """
        return self._prefix + remote_topic
