#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tracer_msgs.msg import TracerStatus

class TracerControlPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tracer_control_publisher', anonymous=True)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Publisher for processed TracerStatus data
        self.status_pub = rospy.Publisher('/tracer_status_processed', TracerStatus, queue_size=10)
        
        # Subscriber for TracerStatus
        rospy.Subscriber('/tracer_status', TracerStatus, self.tracer_status_callback)
        
        # Set the loop rate (in Hz)
        self.rate = rospy.Rate(1)  # 10Hz
        
        # Create a Twist message for velocity commands
        self.velocity_msg = Twist()
        self.velocity_msg.linear.x = 0.2  # Move forward at 0.2 m/s
        self.velocity_msg.angular.z = 0.01  # No angular velocity

    def tracer_status_callback(self, status_msg):
        # This function is called whenever a new TracerStatus message is received
        rospy.loginfo("Received TracerStatus:")
        rospy.loginfo("Linear Velocity: %f m/s", status_msg.linear_velocity)
        rospy.loginfo("Angular Velocity: %f rad/s", status_msg.angular_velocity)
        rospy.loginfo("Battery Voltage: %f V", status_msg.battery_voltage)
        
        # Log motor states
        for i, motor_state in enumerate(status_msg.motor_states):
            rospy.loginfo("Motor %d State:", i)
            rospy.loginfo("  Current: %f A", motor_state.current)
            rospy.loginfo("  RPM: %f", motor_state.rpm)
            rospy.loginfo("  Temperature: %f C", motor_state.temperature)
            rospy.loginfo("  Driver State: %d", motor_state.driver_state)
        
        rospy.loginfo("-----------------------------")
        
        # Publish the received TracerStatus data to a new topic
        self.status_pub.publish(status_msg)

    def run(self):
        while not rospy.is_shutdown():
            # Publish the velocity command
            self.cmd_vel_pub.publish(self.velocity_msg)
            rospy.loginfo("Publishing velocity command: linear.x = %f, angular.z = %f", 
                          self.velocity_msg.linear.x, self.velocity_msg.angular.z)
            
            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        tracer_control_publisher = TracerControlPublisher()
        tracer_control_publisher.run()
    except rospy.ROSInterruptException:
        pass