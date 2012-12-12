Forwards ROS messages between two ROS systems.

mros consists of two processes, the mros_child and the
mros_parent. Typically, the mros_child will run as a server on the
robot. It simply waits for configuration requests. When the
mros_parent is run, it sends configuration requests to the mros_child
(using a ZeroMQ request/reply socket). Using configuration requests,
the mros_parent can get a list of topics currently advertised on the
ROS system to which mros_child is connect; it can tell mros_child what
topics to advertise, and it can tell mros_child what topics it should
subscribe to (and forward to the parent).

After mros_parent configures mros_child, the two each wait for
incoming ROS messages on their ROS system, and forward them to each
other over two ZeroMQ pub/sub connections.

For now, both processes are launched and configured by a single script
called mros_pair. This means that they actually run on the same
computer for the moment, but because they use ZeroMQ for communication
they can easily be split and run on different systems. To run mros_child
and mros_parent, do:

```bash
roscd mros
rosrun mros mros_pair http://localhost:11312 http://localhost:11313 config/test.conf
```

Here the first URI is the ROS_MASTER_URI of the "parent" ROS system, and the second uri
is the ROS_MASTER_URI of the "child" ROS system. test.conf is a json file which
specifies what topics to forward, and what prefix the parent should add to the topics.
