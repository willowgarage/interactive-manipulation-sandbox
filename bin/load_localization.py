#!/usr/bin/python

from subprocess import *
import re, os

POSE_FILE = os.path.join(os.environ['HOME'], 'state/initial_pose.yml')

print "Loading localization from %s" % POSE_FILE
call("rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped -f %s" %
	POSE_FILE, shell=True)

