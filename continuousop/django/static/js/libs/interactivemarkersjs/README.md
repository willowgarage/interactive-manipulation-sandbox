InteractiveMarkers.js provides an implementation of [Interactive Markers](http://www.ros.org/wiki/interactive_markers) 
for the web based on [ros.js](https://github.com/RobotWebTools/rosjs) and 
[three.js](https://github.com/mrdoob/three.js/).

In order to reduce data traffic, it requires you to run a 
[proxy node](https://github.com/dgossow/interactive_marker_proxy) on the topic that you are subscribing to.

The library is designed to be modular, lightweight and to follow the UMD convention. 

Modules
=======
 * markersthree.js: Visualizes Marker messages in three.js
 * threeinteraction.js: Mouse Interaction library for three.js
 * improxy.js: Client to an Interactive Marker proxy server via ros.js
 * imthree.js: View Interactive Markers using the above libraries

Usage
=====

For example WebGL apps, see the examples directory.

To run the basic_controls example, go to the examples directory and run (in two terminals):

`./simple_webserver.py`

`roslaunch basic_controls.launch`