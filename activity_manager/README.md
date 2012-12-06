Installation
=============

The activity manager uses the PLY package to parse queries. First, install ply.

```bash
sudo apt-get install python-ply
```

Next build the activity manager.

```bash
rosmake activity_manager
```

Running
=============

```bash
roslaunch pr2_activities_launch activity_manager.launch
```