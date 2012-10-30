#!/usr/bin/env python  
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Kaijen Hsiao

## @package smach_executer
# Loads saved robot localization poses from file, saves them to file, and broadcasts the 
# current pose every second on /robot_pose

import roslib
roslib.load_manifest('smach_executer')
import rospy
import math
import tf2_ros, tf2
import os, pickle
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion


POSE_FILE_DEFAULT = os.path.join(os.environ['HOME'], 'state/initial_pose.pck')

## get x, y, and z fields in the form of a list
def get_xyz(msg):
    return [msg.x, msg.y, msg.z]

## get x, y, z, and w fields in the form of a list
def get_xyzw(msg):
    return [msg.x, msg.y, msg.z, msg.w]

## return a sting to pretty-print a list of floats
def pprint_str(l):
    return " ".join(["%.3f"%x for x in l])

## check if the robot was just started up unlocalized (starts near the map origin)
def check_localized(trans):
    localized = abs(trans[0]) > 1.0 and abs(trans[1]) > 1.0 
    return localized

## get the translation and rotation for child_frame_id in parent_frame_id in the form of lists 
def lookup_transform(listener, parent_frame_id, child_frame_id, time=None, timeout=None):
    if time == None:
        time = rospy.Time.now()
    if timeout == None:
        timeout = rospy.Duration(2.0)

    transform_stamped = listener.lookup_transform(child_frame_id, parent_frame_id, time, timeout)
    return (get_xyz(transform_stamped.transform.translation), get_xyzw(transform_stamped.transform.rotation))

if __name__ == '__main__':

    rospy.init_node('localization_saver')
    listener = tf2_ros.buffer_client.BufferClient('tf2_buffer_server')
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
    current_pose_pub = rospy.Publisher('/robot_pose', PoseWithCovarianceStamped)

    #path+name of pose file
    pose_file = rospy.get_param("~pose_file", POSE_FILE_DEFAULT)   

    #should we save the current robot pose to file every 5 seconds?
    save_pose = rospy.get_param("~save_pose", True)

    #wait for the map frame to exist (which means localization is now running)
    trans = 0
    rot = 0
    count = 0
    while not rospy.is_shutdown():
        try:
            (trans,rot) = lookup_transform(listener, 'map', 'base_link', rospy.Time(0))
            rospy.loginfo("localization_saver: tf2 says trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))
        except (tf2.LookupException, tf2.ConnectivityException) as e:
            rospy.loginfo("localization_saver: tf2 lookup or connectivity exception, map frame probably not available yet")
            rospy.loginfo("tf2 error: " + e)
            rospy.sleep(5.0)
            continue
        except tf2.TimeoutException:
            rospy.loginfo("localization_saver: tf2 lookup returned TimeoutException")
            continue
        break

    #if robot is not already localized, get the old localized pose from file and broadcast it
    if not check_localized(trans):
        rospy.loginfo("localization_saver: loading localization from %s" % pose_file)
        try:
            if os.path.exists(pose_file):
                infile = open(pose_file, 'r')
                (trans, rot) = pickle.load(infile)
                rospy.loginfo("localization_saver: loaded trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))                
                infile.close()
        except:
            rospy.loginfo("localization_saver: exception when trying to load old localization")
            raise
            
        #broadcast the saved localization as the robot's initial pose
        if check_localized(trans):
            rospy.loginfo("localization_saver: broadcasting saved pose, trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "/map"
            pose_msg.pose.pose.position = Point(*trans)
            pose_msg.pose.pose.orientation = Quaternion(*rot)
            pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            #rospy.loginfo("localization_saver: publishing pose_msg: %s"%str(pose_msg))
            initial_pose_pub.publish(pose_msg)

    # Save and publish the current transform every second
    while not rospy.is_shutdown():
        rospy.sleep(1.0)

        try:
            (trans,rot) = lookup_transform(listener, 'map', 'base_link', rospy.Time(0))

            # If the current transform is not localized, ignore it
            if not check_localized(trans):                
                continue

            # Save the current robot pose (if desired)
            if save_pose:
                rospy.loginfo("localization_saver: saving trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))
                try:
                    if not os.path.exists(os.path.dirname(pose_file)):
                        os.mkdir(os.path.dirname(pose_file))
                    outfile = open(pose_file, 'w')
                    pickle.dump((trans,rot), outfile)
                    outfile.close()
                except Exception, e:
                    rospy.loginfo("localization_saver: Error saving to file")

            # Broadcast the current location on /robot_pose
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.pose.position = Point(*trans)
            pose_msg.pose.pose.orientation = Quaternion(*rot)
            pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            #rospy.loginfo("localization_saver: publishing pose_msg: %s"%str(pose_msg))
            current_pose_pub.publish(pose_msg)

        except (tf2.LookupException, tf2.ConnectivityException) as e:
            rospy.loginfo("localization_saver: tf2 lookup or connectivity exception: " + e)
            continue
        except tf2.TimeoutException:
            rospy.loginfo("localization_saver: tf2 lookup returned TimeoutException")
            continue

