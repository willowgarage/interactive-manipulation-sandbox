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

def check_localized(trans):
    localized = abs(trans[0]) > 1.0 and abs(trans[1]) > 1.0 
    return localized

if __name__ == '__main__':

    rospy.init_node('localization_saver')
    listener = tf.TransformListener()
    pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)

    #wait for the map frame to exist
    trans = 0
    rot = 0
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/odom_combined', rospy.Time(0))
            rospy.loginfo("localization_saver: tf says trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("localization_saver: map frame not available yet")
            rospy.sleep(5.0)
            continue
        break

    #if robot is not localized, get the old localized pose from file and broadcast it
    if not check_localized(trans):
        rospy.loginfo("localization_saver: loading localization from %s" % POSE_FILE)
        try:
            if os.path.exists(POSE_FILE):
                infile = open(POSE_FILE, 'r')
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
                                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            #rospy.loginfo("localization_saver: publishing pose_msg: %s"%str(pose_msg))
            pose_pub.publish(pose_msg)

    #save the current transform every few seconds
    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/map', '/odom_combined', rospy.Time(0))
            if check_localized(trans):
                #rospy.loginfo("localization_saver: saving trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))
                outfile = open(POSE_FILE, 'w')
                pickle.dump((trans,rot), outfile)
                outfile.close()

        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("localization_saver: tf exception")
            continue

        rospy.sleep(5.0)
