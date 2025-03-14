#!/usr/bin/env python3

import rospy
from tracer_msgs.msg import UartTracerStatus  # Import the message type

def tracer_status_publisher():
    # Initialize some node
    rospy.init_node('tracer_status_publisher', anonymous=True)

    # Get parameters
    port_name = rospy.get_param("~port_name", "can0")
    odom_frame = rospy.get_param("~odom_frame", "odom")
    base_frame = rospy.get_param("~base_frame", "base_link")
    control_rate = rospy.get_param("~control_rate", 1)
    simulated_robot = rospy.get_param("~simulated_robot", True)

    # Create a publisher for /tracer_status
    pub = rospy.Publisher('/uart_tracer_status', UartTracerStatus, queue_size=10)

    # some motion commands
    linear_velocity = 0.8  # Example linear velocity
    angular_velocity = 0.1  # Example angular velocity

    # Publish messages at the given control rate
    rate = rospy.Rate(control_rate)  # Hz
    while not rospy.is_shutdown():
        # Create a MotionStatus message
        msg = UartTracerStatus()
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