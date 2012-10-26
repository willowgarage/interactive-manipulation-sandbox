#!/usr/bin/env python  
import roslib
roslib.load_manifest('smach_executer')
import rospy
import math
import tf
import os, pickle
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion

POSE_FILE_DEFAULT = os.path.join(os.environ['HOME'], 'state/initial_pose.pck')

def pprint_str(l):
    return " ".join(["%.3f"%x for x in l])

def check_localized(trans):
    localized = abs(trans[0]) > 1.0 and abs(trans[1]) > 1.0 
    return localized

if __name__ == '__main__':

    rospy.init_node('localization_saver')
    listener = tf.TransformListener()
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
    current_pose_pub = rospy.Publisher('/robot_pose', PoseWithCovarianceStamped)

    #path+name of pose file
    pose_file = rospy.get_param("~pose_file", POSE_FILE_DEFAULT)   

    #should we save the current robot pose to file every 5 seconds?
    save_pose = rospy.get_param("~save_pose", True)

    #wait for the map frame to exist (which means localization is now running)
    trans = 0
    rot = 0
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            rospy.loginfo("localization_saver: tf says trans: %s, rot: %s"%(pprint_str(trans), pprint_str(rot)))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("localization_saver: map frame not available yet")
            rospy.sleep(5.0)
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

    # Save and publish the current transform every five seconds
    while not rospy.is_shutdown():
        rospy.sleep(1.0)

        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

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
            current_pose_pub.publish(pose_msg)

        except (tf.LookupException, tf.ConnectivityException) as e:
            rospy.loginfo("localization_saver: tf exception: " + e)
            continue

