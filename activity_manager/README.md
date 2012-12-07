The activity manager is the successor to smach_executer. Currently, it has
rudimentary ability to run activities, and a query system which will be
used to determine transitions in a state machine of activities. This
README explains the basics of the query system.

Installation
=============

The activity manager uses the PLY package to parse queries. First, install ply:

```bash
sudo apt-get install python-ply
```

Next build the activity manager:

```bash
rosmake activity_manager
```

Running
=============

Launch the acitivity manager server:

```bash
roslaunch pr2_activities_launch activity_manager.launch
```

A simple query
--------------

Try asking the acitivity manager to evaluate a simple query
about whether a given point is within a bounding box:

```bash
rosrun activity_manager query_client simple_within_bbox
```

The script outputs a JSON representation of the actionlib goal sent to
the activity manager, and then the result of the query.  As you can
see, the query goal consists of 2 things:

- query - the query string. this looks like lisp.
- args - a list of argument bindings for the query string.

To keep things simple, there are only variable names in the query
string itself; no literals. For this case, the query string looks like:

```lisp
(point_within_bbox point bbox_origin bbox_size)
```

Each element of the args array specifies the name of one of the
variables in the query string, and the value to assign to that
variable as a JSON string.

More complex queries
--------------

Try running a slightly more complex query:

```bash
rosrun activity_manager query_client conjunction
```

This query returns true if the robot is currently charging and if
a given point is within a bounding box. The query string is:

```lisp
(and (is_charging) (point_within_bbox point bbox_origin bbox_size))
```

Here the function "and" is used with arguments which themselves are
query functions. This ability to nest queries makes it possible
to construct complex queries which can be stored as strings.

The "(is_charging)" function queries the current state of the robot.
It returns a boolean. There are other queries which query the robot's
state. For example, to find out the transform between two TF frames:

```bash
rosrun activity_manager query_client simple_get_transform
```

This runs the query:

```lisp
(get_transform frame_a frame_b)
```

With frame_a set to "r_wrist_roll_link" and frame_b set to "base_link".
The idea is that people can combine queries about transforms with
queries about bounding boxes to find out if a certain part of the robot
is in a specified area.

To see a list of the currently implemented functions and their argument
types, run:

```bash
rosrun activity_manager list_query_functions
```