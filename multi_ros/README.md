Installation
==========

```bash
sudo apt-get install python-zmq
```

Description
==========

Forwards ROS messages between two ROS systems.

multi_ros consists of two processes, the multi_ros_client and the
multi_ros_server. Typically, the multi_ros_client will run as a server on the
robot. It simply waits for configuration requests. When the
multi_ros_server is run, it sends configuration requests a number of multi_ros_clients
(using a ZeroMQ request/reply socket). Using configuration requests,
the multi_ros_parent can get a list of topics currently advertised on the
ROS system to which multi_ros_client is connected; it can tell multi_ros_client what
topics to advertise, and it can tell multi_ros_child what topics it should
subscribe to (and forward to the parent).

After multi_ros_server configures the multi_ros_clients, it waits for
incoming ROS messages on their ROS system, and forwards them to each
other over two ZeroMQ pub/sub connections.

Configuration
==========
Config files are in json format as the following example

{
"uri": "tcp://clientname",
"prefix": "/uniquename",
"topics": [
   {"topic": "/chatter", "message_type": "std_msgs/String", "compression": "zlib", "rate": 1.0},
   {"topic": "/rosout" , "message_type": "rosgraph_msgs/Log", "compression": "zlib", "rate": 1.0}
   ]
}

uri
Should be a tcp protocol and hostname

prefix
This prefix should be unique

topics
Any number of topics can be forwarded
topic - name of topic
message_type - ros message type of topic
compression - zlib

prefix


Use
==========

On server:
```bash
roscd multi_ros
rosrun multi_ros multi_ros_server config/test.conf config/prn.conf
```

On all clients(robots):
```bash
rosrun multi_ros multi_ros_client
```
