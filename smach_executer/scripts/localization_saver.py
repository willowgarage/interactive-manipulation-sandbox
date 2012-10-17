#!/usr/bin/env python  
import roslib
roslib.load_manifest('smach_executer')
import rospy
import math
import tf
import os, pickle
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion

POSE_FILE = os.path.join(os.environ['HOME'], 'state/initial_pose.yml')

def pprint_str(l):
    return " ".join(["%.3f"%x for x in l])

if __name__ == '__main__':

    rospy.init_node('localization_saver')
    listener = tf.TransformListener()

    #wait for the map frame to exist
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/odom_combined', rospy.Time(0))
            print "trans: ", pprint_str(trans)
            print "rot: ", pprint_str(rot)
        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("map frame not available yet")
            rospy.sleep(1.0)
            continue
        break

    #if robot is not localized, get the old localized pose from file and broadcast it
    if trans[0] == 0:
        rospy.loginfo("Loading localization from %s" % POSE_FILE)
        try:
            if os.path.exists(POSE_FILE):
                infile = open(POSE_FILE, 'r')
                (trans, rot) = pickle.load(pfile)
                pprint((trans, rot))
                infile.close()
        except:
            rospy.loginfo("exception when trying to load old localization")
            
        #broadcast the saved localization as the robot's initial pose
        if trans != None and rot != None:
            rospy.loginfo("broadcasting saved pose, trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))
            pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "/map"
            pose_msg.pose.position = Point(*trans)
            pose_msg.pose.orientation = Quaternion(*rot)
            pose_pub.publish(pose_msg)

    #save the current transform every few seconds
    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/map', '/odom_combined', rospy.Time(0))
            #rospy.loginfo("localization_saver: saving trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))
            if trans[0] != 0:
                outfile = open(POSE_FILE, 'w')
                pickle.dump((trans,rot), outfile)
                outfile.close()

        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("localization_saver: tf exception")
            continue

        rospy.sleep(5.0)

