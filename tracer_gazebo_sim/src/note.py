#!/usr/bin/env python3

import rospy
from tracer_msgs.msg import TracerStatus  # Message type for /tracer_status
from nav_msgs.msg import Odometry  # Message type for /odom

# Global variables to store robot velocity
linear_velocity = 0.3
angular_velocity = 0.3

# Callback function to update velocities from /odom
def odom_callback(msg):
    global linear_velocity, angular_velocity
    linear_velocity = msg.twist.twist.linear.x
    angular_velocity = msg.twist.twist.angular.z

def tracer_status_publisher():
    global linear_velocity, angular_velocity

    # Initialize the node
    rospy.init_node('tracer_status_publisher_baq', anonymous=True)

    # Get parameters
    control_rate = rospy.get_param("~control_rate", 1)

    # Create a publisher for /uart_tracer_status
    pub = rospy.Publisher('/uart_tracer_status', TracerStatus, queue_size=10)

    # Subscribe to /odom to get real robot velocities
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publish messages at the given control rate
    rate = rospy.Rate(control_rate)  # Hz
    while not rospy.is_shutdown():
        # Create a MotionStatus message
        msg = TracerStatus()
        msg.linear_velocity = linear_velocity
        msg.angular_velocity = angular_velocity

        # Log and publish the message
        rospy.loginfo(f"Publishing: linear={msg.linear_velocity}, angular={msg.angular_velocity}")
        pub.publish(msg)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == "__main__":
    try:
        tracer_status_publisher()
    except rospy.ROSInterruptException:
        pass
