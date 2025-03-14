#!/usr/bin/env python3

import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

class DirectController:
    def __init__(self):
        rospy.init_node('direct_controller')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Subscribe to RViz goal
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Current goal
        self.current_goal = None
        
        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.2  # m
        
        # Initialize the robot at origin
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0
        
        # Create the mandatory TF transforms
        self.publish_tf_frames()
        
        rospy.loginfo("Direct controller initialized and ready")
    
    def publish_tf_frames(self):
        """Publish the necessary TF transforms."""
        # Publish map -> odom transform (fixed)
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "odom",
            "map"
        )
        
        # Publish odom -> base_link transform (based on robot position)
        self.tf_broadcaster.sendTransform(
            (self.robot_x, self.robot_y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.robot_theta),
            rospy.Time.now(),
            "base_link",
            "odom"
        )
    
    def goal_callback(self, msg):
        """Handle the goal from RViz."""
        # Extract goal position
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        # Extract goal orientation
        q = msg.pose.orientation
        _, _, goal_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        rospy.loginfo(f"Received goal: ({goal_x}, {goal_y}, {goal_theta})")
        
        # Set as current goal
        self.current_goal = (goal_x, goal_y, goal_theta)
        
        # Visualize the goal
        self.visualize_goal(goal_x, goal_y)
        
        # Start moving to the goal
        self.move_to_goal()
    
    def visualize_goal(self, x, y):
        """Publish a marker to visualize the goal."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goals"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)
    
    def move_to_goal(self):
        """Move the robot to the current goal."""
        if not self.current_goal:
            return
        
        goal_x, goal_y, goal_theta = self.current_goal
        
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and self.current_goal:
            # Calculate distance to goal
            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if we've reached the goal
            if distance < self.position_tolerance:
                rospy.loginfo("Reached goal position!")
                self.stop_robot()
                # Clear the goal
                self.current_goal = None
                break
            
            # Calculate angle to goal
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference
            angle_diff = target_angle - self.robot_theta
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Control command
            cmd = Twist()
            
            # If we need to turn a lot, just turn
            if abs(angle_diff) > 0.3:
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                rospy.loginfo_throttle(1, f"Turning to face goal: angle_diff={angle_diff:.2f}")
            else:
                # We're roughly facing the goal, move forward and adjust orientation
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.5 * angle_diff  # Proportional control
                rospy.loginfo_throttle(1, f"Moving to goal: distance={distance:.2f}, angle_diff={angle_diff:.2f}")
            
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
            # Update robot position (simple integration)
            # In a real system, this would come from odometry
            self.robot_theta += cmd.angular.z * 0.1  # dt = 0.1s
            self.robot_x += cmd.linear.x * math.cos(self.robot_theta) * 0.1
            self.robot_y += cmd.linear.x * math.sin(self.robot_theta) * 0.1
            
            # Update the TF frames with the new position
            self.publish_tf_frames()
            
            rate.sleep()
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def run(self):
        """Main run loop."""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Keep publishing TF frames
            self.publish_tf_frames()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = DirectController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 