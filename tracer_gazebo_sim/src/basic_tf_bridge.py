#!/usr/bin/env python3

import rospy
import tf
import time
from nav_msgs.msg import Odometry

def main():
    rospy.init_node('basic_tf_bridge')
    
    # Create a broadcaster
    broadcaster = tf.TransformBroadcaster()
    
    # Get the current time
    now = rospy.Time.now()
    
    # First, publish a static transform from map to odom
    broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        now,
        "odom",
        "map"
    )
    
    # Sleep to let the transform propagate
    rospy.sleep(1.0)
    
    # Log what we've done
    rospy.loginfo("Published initial map -> odom transform")
    
    # Create a listener to check what frames exist
    listener = tf.TransformListener()
    rospy.sleep(2.0)  # Give it time to build up the TF tree
    
    # Check what frames exist
    frames = listener.getFrameStrings()
    rospy.loginfo("Available TF frames: %s", frames)
    
    # Function to handle odometry messages
    def odom_callback(msg):
        # Republish the odom -> base_link transform from the odometry message
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        
        broadcaster.sendTransform(
            (pos.x, pos.y, pos.z),
            (quat.x, quat.y, quat.z, quat.w),
            msg.header.stamp,
            "base_link",
            "odom"
        )
        
        # Also republish map -> odom
        broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            msg.header.stamp,
            "odom",
            "map"
        )
    
    # Subscribe to odometry
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # Publish static transforms at regular intervals as a fallback
    r = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "odom",
            "map"
        )
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 