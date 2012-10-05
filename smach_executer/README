Installation
============

Install interactive-manipulation-sandbox, object_manipulation, pr2_object_manipulation,
pr2_plugs, and smach from source. Here’s the lines for your rosinstall (modify if you’re
using your own forks of any repos):

- hg: {local-name: src/pr2_plugs, uri: 'https://kforge.ros.org/plugs/hg'}
- hg: {local-name: src/smach, uri: 'https://kforge.ros.org/smach/executive_smach'}
- git: {local-name: src/object_manipulation, uri: 'git@github.com:ros-interactive-manipulation/object_manipulation.git'}
- git: {local-name: src/pr2_object_manipulation, uri: 'git@github.com:ros-interactive-manipulation/pr2_object_manipulation.git'}
- git: {local-name: src/interactive-manipulation-sandbox, uri: 'git@github.com:willowgarage/interactive-manipulation-sandbox.git'}

Build everything:

    rosmake pr2_interactive_manipulation pr2_plugs_actions smach_executer



Running the Executer Server
==========================

First launch interactive manipulation with the plugs, wg, and nav arguments:

    roslaunch pr2_interactive_manipulation pr2_interactive_manipulation_robot.launch plugs:=true wg:=true nav:=true

Then run the executer server:

    rosrun smach_executer executer



Executing Actions
=================

There are pre-written example json descriptions for executing some actions in smach_executer/test/json.
These can be sent to the server manually to make sure it is working by running:

    rosrun smach_executer call_executer <json file>



Documentation for the Actions
=============================

For the moment, actions are documented in the doc strings of their corresponding
python classes. You can find these classes, one per file, in src/smach_executer/actions
