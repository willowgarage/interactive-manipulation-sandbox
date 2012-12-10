class MRosParent:
    def __init__(self, child_config_uri, child_sub_uri, child_pub_rui, prefix='', poll_rate=1.):
        '''
        Args:

        child_config_uri (str) - uri at which child will answer config requests.
        child_sub_uri (str) - uri at which child will subscribe to messages.
        child_pub_uri (str) - uri at which child will publish messages.
        '''
        self._topics = {}
        self._topics_lock = threading.Lock()
        self._prefix = prefix
        self._poll_rate = poll_rate

        # create class for interfacing with the ROS master
        caller_id = rospy.get_caller_id()
        self._master = rosgraph.Master(caller_id)

        self._zmq_context = zmq.Context()

        # socket for setting config on child
        self._zmq_config_socket = self._zmq_context.socket(zmq.REQ)
        self._zmq_config_socket.connect(child_config_socket_uri)

        # socket for sending ROS messages to child
        self._zmq_pub_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_pub_socket.connect(child_sub_socket_uri)

        # socket for receiving ROS messages from child
        self._zmq_sub_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_sub_socket.connect(child_pub_socket_uri)

    def run(self):
        self._local_update_thread = threading.Thread(target=self.local_update_thread_func)
        self._remote_update_thread = threading.Thread(target=self.remote_update_thread_func)
        self._local_update_thread.start()
        self._remote_update_thread.start()        
        rospy.spin()

    def received_ros_message_cb(self, msg, topic):
        # TODO: throttle based on priority
        self._zmq_data_socket.send((topic, msg))

    def local_update(self):
        '''
        Checks the ROS master for list of topics, as well as number of
        subscribers and publishers for each topic.
        '''
        # get all topics from the ROS master, and a list of publishers and subscribers for each
        pubs, subs, services = self._master.getSystemState()
        pubs = dict(pubs)
        subs = dict(subs)
        all_local_topics = set.union(set(pubs.keys()), set(subs.keys()))

        # get the types of all topics from the ROS master
        topic_types = dict(self._master.getTopicTypes())
        
        with self._topics_lock:
            # clear previous info on local subscribers/publishers
            for topic in self._topics:
                self.num_local_subscribers = 0
                self.num_local_publishers = 0

            # update info on publishers
            for topic in all_local_topics:
                if not topic in self._topics:
                    self._topics[topic] = TopicInfo(topic)
                topic_info = self._topics[topic]

                # set the topic's message type (this is just a string)
                topic_info.message_type = topic_types[topic]

                # this is getting the md5sum from the message class, not from the node
                # publishing the topic. we're assuming that it is using the same message
                # definition as we find in our ROS_PACKAGE_PATH
                message_class = roslib.message.get_message_class(topic_info.message_type)
                if message_class is None:
                    print 'Warning: no message class for message type %s' % topic_info.message_type
                    continue
                topic_info.md5sum = message_class._md5sum
                    
                if topic in pubs:
                    # TODO: remove self from list of publishers                    
                    topic_info.num_local_publishers = len(pubs[topic])

                if topic in subs:
                    # TODO: remove self from list of subscribers
                    topic_info.num_local_subscribers = len(subs[topic])

    def remote_update(self, update_dict):
        with self._topics_lock:
            for topic in update_dict:
                if not topic in self._topics:
                    self._topics[topic] = TopicInfo(topic)

                topic_info = self._topics[topic]
                    
                topic_info.num_remote_subscribers = update_dict[topic]['num_subscribers']
                topic_info.num_remote_publishers = update_dict[topic]['num_publishers']
                topic_info.message_type = update_dict[topic]['message_type']
                topic_info.md5sum = update_dict[topic]['md5sum']

                # all topics that exist on the remote system should have local publishers, so that
                # they are visible on the local ROS master
                if topic_info.publisher is None:
                    passthrough_message_class = make_passthrough_message_class(topic_info.message_type, topic_info.md5sum)
                    #topic_info.publisher = rospy.Publisher(topic, passthrough_message_class)

                # if there is a subscriber on the remote side, there needs to be a subscriber
                # on the local side, so that we can forward along all messages on this topic
                # TODO: destroy local subscriber when there are no more remote subscribers
                if topic_info.subscriber is None and topic_info.num_remote_subscribers > 0:
                    passthrough_message_class = make_passthrough_message_class(topic_info.message_type, topic_info.md5sum)
                    topic_info.subscriber = rospy.Subscriber(topic, passthrough_message_class)
                    
    def local_update_thread_func(self):
        poll_rate = rospy.Rate(self._poll_rate)
        while not rospy.is_shutdown():
            print 'Running local update'
            self.local_update()

            # send an update to the remote
            update_dict = {}            
            with self._topics_lock:
                for topic in self._topics:
                    topic_info = self._topics[topic]
                    update_dict[topic] = {
                        'num_subscribers' : topic_info.num_local_subscribers,
                        'num_publishers' : topic_info.num_local_publishers,
                        'md5sum' : topic_info.md5sum,
                        'message_type' : topic_info.message_type
                        }
            self._zmq_config_socket.send(pickle.dumps(update_dict))

            # send the list
            poll_rate.sleep()

    def remote_update_thread_func(self):
        while not rospy.is_shutdown():
            print 'Running remote update'
            # get a config message from the remote
            update_dict = pickle.loads(self._zmq_config_socket.recv())
            self.remote_update(update_dict)
