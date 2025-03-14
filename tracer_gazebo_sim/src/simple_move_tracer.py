#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_robot():

    #initialize a node
    rospy.init_node('move_tracer_node',anonymous=True)

    #create a publiser to cmd_vel
    cmd_vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    #create the loop rate
    rate=rospy.Rate(1) #every second

    #Create a Twist message and set the velocity
    velocity_msg=Twist()
    velocity_msg.linear.x=0.5
    velocity_msg.angular.z=0.1


    def shutdown_hook():
        rospy.loginfo("stopping the robot...")
        stop_msg=Twist()
        stop_msg.linear.x=0.0
        stop_msg.angular.z=0.0
        cmd_vel_pub.publish(stop_msg)
        rospy.sleep(1)

    rospy.on_shutdown(shutdown_hook)

    # keep publising the velocity command in the loop and rate
    rospy.loginfo("moving the robot with 0.2 velocity")
    while not rospy.is_shutdown():
        cmd_vel_pub.publish(velocity_msg)
        rate.sleep()

if __name__=='__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass



    