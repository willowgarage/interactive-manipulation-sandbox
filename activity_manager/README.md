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

Each element of the argument array 


