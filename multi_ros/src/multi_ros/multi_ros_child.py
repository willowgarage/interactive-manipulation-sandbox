import zmq
import threading
import rospy
import cPickle as pickle

from multi_ros.multi_ros_node import MultiRosNode
from multi_ros.published_topic import PublishedTopic
from multi_ros.subscribed_topic import SubscribedTopic


class MultiRosChild(MultiRosNode):
    def __init__(self, child_uri='tcp://*', name='MultiRosChild', child_ros_master_uri=None, poll_rate=1.0):
        super(MultiRosChild, self).__init__(name, child_ros_master_uri, poll_rate)

        config_uri = child_uri + ':5000'
        sub_uri = child_uri + ':5001'
        pub_uri = child_uri + ':5002'
        rospy.loginfo('%s binding to %s for conf' % (self._name, config_uri))
        self._zmq_config_socket = self._zmq_context.socket(zmq.REP)
        self._zmq_config_socket.bind(config_uri)
        rospy.loginfo('%s binding to %s for pub' % (self._name, pub_uri))
        self._zmq_pub_socket.bind(pub_uri)
        rospy.loginfo('%s binding to %s for sub' % (self._name, sub_uri))
        self._zmq_sub_socket.bind(sub_uri)

        # start thread to forward remote messages
        self._remote_message_thread = threading.Thread(target=self.remote_forwarding_loop)
        self._remote_message_thread.start()

        self.config_listener_loop()

    def config_listener_loop(self):
        loop_rate = rospy.Rate(self._poll_rate)
        while not rospy.is_shutdown():
            msg_str = self._zmq_config_socket.recv()
            req = pickle.loads(msg_str)
            cmd = req['COMMAND']
            if cmd == 'GET_TOPIC_LIST':
                self._ros_interface.update_ros_graph()
                rospy.loginfo('%s: sending topic list' % self._name)
                local_topics = self._ros_interface.get_ros_graph()
                self._zmq_config_socket.send(pickle.dumps(local_topics))
            elif cmd == 'ADVERTISE':
                for topic_dict in req['TOPICS']:
                    topic_info = PublishedTopic(str(topic_dict['topic']), str(topic_dict['message_type']),
                                                str(topic_dict['md5sum']), topic_dict.get('compression', None))
                    rospy.loginfo('%s advertising %s' % (self._name, topic_info.topic))
                    self._ros_interface.advertise(topic_info.topic, topic_info.message_type, topic_info.md5sum)
                    with self._pub_topics_lock:
                        self._pub_topics[topic_info.topic] = topic_info
                self._zmq_config_socket.send(pickle.dumps('OK'))
            elif cmd == 'SUBSCRIBE':
                for topic_dict in req['TOPICS']:
                    topic_info = SubscribedTopic(str(topic_dict['topic']), str(topic_dict['message_type']),
                                                 str(topic_dict['md5sum']), topic_dict.get('rate', None),
                                                 topic_dict.get('compression', None))
                    rospy.loginfo('%s subscribing to %s' % (self._name, topic_info.topic))
                    self._ros_interface.subscribe(topic_info.topic, topic_info.message_type, topic_info.md5sum)
                    with self._sub_topics_lock:
                        self._sub_topics[topic_info.topic] = topic_info
                self._zmq_config_socket.send(pickle.dumps('OK'))
            else:
                rospy.logerr('Unknown command %s' % cmd)
                self._zmq_config_socket.send(pickle.dumps('ERROR'))

            loop_rate.sleep()
