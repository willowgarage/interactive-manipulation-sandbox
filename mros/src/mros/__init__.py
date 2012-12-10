import time, threading
import cPickle as pickle
from new import classobj
import zmq
import rospy, rostopic, rosgraph, roslib

def make_passthrough_message_class(message_type, md5sum):
    '''
    Creates a message class which knows its type, but doesnt serialize
    or deserialize.

    Args:
        message_type (str) : ROS message type.
    '''
    message_class = classobj('t_passthrough_%s' % message_type, (rospy.msg.AnyMsg,), {
        '_type' : message_type,
        '_md5sum' : md5sum
        })
    return message_class

class TopicInfo:
    '''
    Contains information about a single topic.

    This topic may exist on the remote ROS system, the local
    ROS system, or both.
    '''
    def __init__(self, local_topic):
        self.local_topic = local_topic
        self.message_type = None
        self.md5sum = None
        self.num_local_subscribers = 0
        self.num_local_publishers = 0
        self.num_remote_subscribers = 0
        self.num_remote_publishers = 0
        self.publisher = None
        self.subscriber = None

class MRos:
