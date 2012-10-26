This demo app runs the Interactive Markers basic_controls tuturial in the browser.

You need to get the rosbridge suite first:

https://github.com/RobotWebTools/rosbridge_suite

To compile, add the interactive-manipulation-sandbox repo to your ROS_PACKAGE_PATH and run

`rosmake demo_interactive_markers`

To run, launch rosbridge, rosapi and the interactive marker tunnel (im_tunnel)

`roslaunch demo_interactive_markers main.launch`

Then, open www/interactive_markers.html.
