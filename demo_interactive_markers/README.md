This demo app runs the Interactive Markers basic_controls tutorial in the browser.

Installation
============

Install interactive_markers and rviz from source:

 1. `cd ~/ws/`
 2. `wget https://raw.github.com/gist/4021009/cb48243596cae5cf3aa5e9b6abea4487ab09e08d/gistfile1.txt`
 3. `rosinstall .`
 4. `source ~/ws/setup.bash`

Install rosbridge suite:

 1. `sudo apt-get install ros-fuerte-rosbridge-suiteq

Compile and run interactive markers demo

 1. `export ROS_PACKAGE_PATH=/path/to/demo_interactive_markers/:$ROS_PACKAGE_PATH`
 2. `rosmake demo_interactive_markers`
 3. `roslaunch demo_interactive_markers main.launch`

View in browser by going to
file:///path/to/demo_interactive_markers/www/interactive_markers.html.

