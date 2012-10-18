#!/usr/bin/python
import numpy as np

# Uses: http://en.wikipedia.org/wiki/Affine_transformation#Augmented_matrix

known_mappings = [
    # All arrays are: [x, y]
#    { 'pose': [ 21.661, 25.623 ], 'maploc': [ 280, 200 ]},  # Power outlet in the Green Room
#    { 'pose': [ 9.304, 29.383 ], 'maploc': [ 300, 305 ]},   # Power outlet outside Julius's and David's ofice
#    { 'pose': [ 7.404, 30.788 ], 'maploc': [ 296, 340 ]}   # Location right outside of Chad's office
    { 'pose': [ 3.227, 20.119 ], 'maploc': [ 399, 338 ]},  # Bottom right corner of map near exit door
    { 'pose': [ 33.492, 5.723 ], 'maploc': [ 412, 50 ]},   # Inside the kitchen
    { 'pose': [ 27.086, 52.780 ], 'maploc': [ 70, 260 ]}   # Slanted wall near Cathedral
]

# Create list of input vectors
# Note each vector has to have an additional
# fake dimension hardcoded to '1' in order to
# enable translation
poses = []
for known_mapping in known_mappings:
    pose = known_mapping['pose']
    pose.append(1)
    poses.append(pose)

# Same with output vectors
maplocs = []
for known_mapping in known_mappings:
    maploc = known_mapping['maploc']
    maploc.append(1)
    maplocs.append(maploc)

# Convert vector lists into numpy objects
a = np.array(poses)
b = np.array(maplocs)

# Solve the system
transf = np.linalg.solve( a, b)

print "Your transformation matrix is:\n %s" % transf

# Decompose the transformation in components

# 1: Traslation
translation = transf[2,0:2]
print "Translation: %s" % translation

lintrans = transf[0:2,0:2]
scale_x = np.sign(transf[0,0])*np.sqrt(transf[0,0]**2+transf[1,0]**2)
scale_y = np.sign(transf[1,1])*np.sqrt(transf[0,1]**2+transf[1,1]**2)
rotation = np.arctan(transf[0,1]/transf[1,1]) * 2.0*np.pi
print "Scale = (%f,%f), Rotation = %f" % (scale_x, scale_y, rotation)

# Verify with an additional point
verify_pose = np.array([9.424, 29.524, 1])
#answer = transf.dot(verify_pose)
answer = verify_pose.dot(transf)

print "Verify point in map is: %s" % answer
