#!/usr/bin/python

from subprocess import *
import re, os

POSE_FILE = os.path.join(os.environ['HOME'], 'state/initial_pose.yml')

# Initialize
data = ''
#- Translation: [9.541, 29.336, 0.000]
xlation = re.compile("Translation: \[([^,]+), ([^,]+), ([^,]+)\]")
#- Rotation: in Quaternion [-0.002, 0.001, 0.962, 0.272]
orient = re.compile("Rotation: in Quaternion \[([^,]+), ([^,]+), ([^,]+), ([^,]+)\]")

# Open a pipe to tf output
p = Popen("rosrun tf tf_echo /map /base_footprint", stdout=PIPE, shell=True, bufsize=1)
data += p.stdout.read(256)

# Loop until we find the coordinates
xlation_values = None
orient_values = None
while True:
	match = xlation.search(data)
	if match:
		xlation_values = match.group(1,2,3)
		break
	data += p.stdout.read(256)

while True:
	match = orient.search(data)
	if match:
		orient_values = match.group(1,2,3,4)
		break
	data += p.stdout.read(256)
	
p.kill()

INITIAL_POSE = """
header: 
  seq: 4
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: /map
pose: 
  pose: 
    position: 
      x: %s
      y: %s
      z: %s
    orientation: 
      x: %s
      y: %s
      z: %s
      w: %s
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
""" % (xlation_values + orient_values)

print 'Current pose:'
print INITIAL_POSE
print "Saving localization information to %s" % POSE_FILE

if not os.path.exists(os.path.dirname(POSE_FILE)):
	os.mkdir(os.path.dirname(POSE_FILE))
f = open(POSE_FILE, 'w')
f.write(INITIAL_POSE)
f.close()
