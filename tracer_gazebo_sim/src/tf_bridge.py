#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class TFBridge:
    def __init__(self):
        rospy.init_node('tf_bridge', anonymous=True)
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # TF2 static broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # TF listener to check if transforms are working
        self.tf_listener = tf.TransformListener()
        
        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Publish static transforms immediately
        self.publish_static_transforms()
        
        # Set up timer to periodically publish static transforms in case they're not received
        self.static_timer = rospy.Timer(rospy.Duration(1.0), self.static_timer_callback)
        
        rospy.loginfo("TF Bridge initialized")
    
    def odom_callback(self, msg):
        """Republish odometry as TF transforms."""
        # Extract translation and rotation from odometry
        trans = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        rot = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        # Publish transform from odom to base_link
        self.tf_broadcaster.sendTransform(
            trans,
            rot,
            msg.header.stamp,
            'base_link',  # Child frame
            'odom'        # Parent frame
        )
        
        # Also republish the map to odom transform each time
        # This ensures both transforms are published close in time
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            msg.header.stamp,
            'odom',
            'map'
        )
    
    def publish_static_transforms(self):
        """Publish static transforms for the robot's TF tree."""
        # Create map to odom transform
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Publish the static transform
        self.static_broadcaster.sendTransform([transform])
        
        # Also publish as a regular (non-static) transform for immediate effect
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            'odom',
            'map'
        )
        
        rospy.loginfo("Published static transforms")
    
    def static_timer_callback(self, event):
        """Periodically publish static transforms to ensure they're active."""
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            'odom',
            'map'
        )
        
        # Check if the transform is properly established
        try:
            self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(0.1))
            connected = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            connected = False
        
        if connected:
            rospy.loginfo_throttle(10, "Transform between map and base_link is established")
        else:
            rospy.logwarn_throttle(5, "No transform between map and base_link yet")
    
    def run(self):
        """Main run loop."""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Republish the static transform periodically
            self.tf_broadcaster.sendTransform(
                (0, 0, 0),
                (0, 0, 0, 1),
                rospy.Time.now(),
                'odom',
                'map'
            )
            
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = TFBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass 