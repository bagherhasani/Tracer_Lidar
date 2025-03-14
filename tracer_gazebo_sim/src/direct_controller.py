#!/usr/bin/env python3

import rospy
import tf
import math
import threading
import time
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from std_msgs.msg import ColorRGBA

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.markers_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher('/waypoint_path', Path, queue_size=1)
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Subscribe to RViz goal
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.2  # m
        
        # Initialize the robot position
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0
        
        # List of waypoints (x, y, theta)
        self.waypoints = []
        
        # Lock for thread-safe access to waypoints
        self.waypoints_lock = threading.Lock()
        
        # Flag to indicate if we're currently navigating
        self.is_navigating = False
        
        # Current waypoint index
        self.current_waypoint_index = 0
        
        # Create the mandatory TF transforms
        self.publish_tf_frames()
        
        # Debug - print diagnostic info periodically
        self.debug_timer = rospy.Timer(rospy.Duration(5.0), self.debug_callback)
        
        rospy.loginfo("=== Waypoint Navigator Ready ===")
        rospy.loginfo("Use the 'Set 2D Nav Goal' button in RViz to set waypoints")
        rospy.loginfo("The robot will visit all waypoints in the order they were set")
    
    def debug_callback(self, event):
        with self.waypoints_lock:
            wp_count = len(self.waypoints)
            current_idx = self.current_waypoint_index
        
        rospy.loginfo("===== NAVIGATOR STATUS =====")
        rospy.loginfo(f"Total waypoints: {wp_count}")
        rospy.loginfo(f"Current waypoint: {current_idx+1 if current_idx < wp_count else 'NONE'}")
        rospy.loginfo(f"Navigation active: {self.is_navigating}")
        rospy.loginfo(f"Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_theta:.2f})")
        rospy.loginfo("===========================")
    
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
        
        # Add the waypoint to our list
        with self.waypoints_lock:
            self.waypoints.append((goal_x, goal_y, goal_theta))
            waypoint_num = len(self.waypoints)
        
        rospy.loginfo(f"Added waypoint #{waypoint_num}: ({goal_x:.2f}, {goal_y:.2f}, {goal_theta:.2f})")
        
        # Visualize all waypoints
        self.visualize_waypoints()
        
        # If we're not already navigating, start navigation
        if not self.is_navigating:
            # Start navigation in a separate thread
            nav_thread = threading.Thread(target=self.navigate_waypoints)
            nav_thread.daemon = True
            nav_thread.start()
            rospy.loginfo("Navigation started")
    
    def visualize_waypoints(self):
        """Publish markers to visualize all waypoints and the path."""
        # Create marker array for waypoints
        marker_array = MarkerArray()
        
        # Create path message
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        with self.waypoints_lock:
            for i, (x, y, theta) in enumerate(self.waypoints):
                # Add waypoint marker
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "waypoints"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.1
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                
                # Current waypoint is red, completed ones are green, upcoming are blue
                if i < self.current_waypoint_index:
                    marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green (completed)
                elif i == self.current_waypoint_index:
                    marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red (current)
                else:
                    marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue (upcoming)
                
                marker_array.markers.append(marker)
                
                # Add direction arrow for each waypoint
                arrow = Marker()
                arrow.header.frame_id = "map"
                arrow.header.stamp = rospy.Time.now()
                arrow.ns = "directions"
                arrow.id = i
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = x
                arrow.pose.position.y = y
                arrow.pose.position.z = 0.1
                q = tf.transformations.quaternion_from_euler(0, 0, theta)
                arrow.pose.orientation.x = q[0]
                arrow.pose.orientation.y = q[1]
                arrow.pose.orientation.z = q[2]
                arrow.pose.orientation.w = q[3]
                arrow.scale.x = 0.5  # Arrow length
                arrow.scale.y = 0.1  # Arrow width
                arrow.scale.z = 0.1  # Arrow height
                arrow.color = marker.color  # Same color as the waypoint
                
                marker_array.markers.append(arrow)
                
                # Add to path
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                path.poses.append(pose)
            
            # Add current robot position to path if we're navigating
            if self.is_navigating and len(self.waypoints) > 0:
                robot_pose = PoseStamped()
                robot_pose.header = path.header
                robot_pose.pose.position.x = self.robot_x
                robot_pose.pose.position.y = self.robot_y
                robot_pose.pose.position.z = 0
                q = tf.transformations.quaternion_from_euler(0, 0, self.robot_theta)
                robot_pose.pose.orientation.x = q[0]
                robot_pose.pose.orientation.y = q[1]
                robot_pose.pose.orientation.z = q[2]
                robot_pose.pose.orientation.w = q[3]
                
                # Insert at the beginning or where appropriate
                if self.current_waypoint_index == 0:
                    path.poses.insert(0, robot_pose)
                elif self.current_waypoint_index < len(path.poses):
                    path.poses.insert(self.current_waypoint_index, robot_pose)
        
        # Publish markers and path
        self.markers_pub.publish(marker_array)
        self.path_pub.publish(path)
    
    def navigate_waypoints(self):
        """Navigate through all waypoints."""
        self.is_navigating = True
        self.current_waypoint_index = 0
        
        rospy.loginfo("Starting waypoint navigation")
        
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and self.is_navigating:
            # Check if we have waypoints to navigate to
            with self.waypoints_lock:
                if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
                    rospy.loginfo("No more waypoints to navigate to")
                    self.is_navigating = False
                    self.stop_robot()
                    break
                
                # Get current waypoint
                current_waypoint = self.waypoints[self.current_waypoint_index]
                
                # Log that we're processing this waypoint
                rospy.loginfo_throttle(5, f"Navigating to waypoint #{self.current_waypoint_index + 1} of {len(self.waypoints)}")
            
            # Navigate to current waypoint
            goal_x, goal_y, goal_theta = current_waypoint
            
            # Calculate distance to goal
            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if we've reached the waypoint
            if distance < self.position_tolerance:
                rospy.loginfo(f"*** Reached waypoint #{self.current_waypoint_index + 1} ***")
                
                # Stop briefly at this waypoint
                self.stop_robot()
                rospy.sleep(0.5)  # Pause briefly
                
                # Move to next waypoint
                with self.waypoints_lock:
                    self.current_waypoint_index += 1
                    
                    # Log the transition to the next waypoint
                    if self.current_waypoint_index < len(self.waypoints):
                        rospy.loginfo(f"Moving to waypoint #{self.current_waypoint_index + 1}")
                    else:
                        rospy.loginfo("All waypoints completed!")
                        self.is_navigating = False
                        break
                
                # Update visualization
                self.visualize_waypoints()
                
                # Continue to next iteration to start processing the next waypoint
                continue
            
            # Calculate angle to goal
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference
            angle_diff = target_angle - self.robot_theta
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Create control command
            cmd = Twist()
            
            # If we need to turn a lot, just turn
            if abs(angle_diff) > 0.3:
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                rospy.loginfo_throttle(1, f"Turning to face waypoint #{self.current_waypoint_index + 1}: angle_diff={angle_diff:.2f}")
            else:
                # We're roughly facing the goal, move forward and adjust orientation
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.5 * angle_diff  # Proportional control
                rospy.loginfo_throttle(1, f"Moving to waypoint #{self.current_waypoint_index + 1}: distance={distance:.2f}, angle_diff={angle_diff:.2f}")
            
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
            # Update robot position (simple integration)
            # In a real system, this would come from odometry
            self.robot_theta += cmd.angular.z * 0.1  # dt = 0.1s
            self.robot_x += cmd.linear.x * math.cos(self.robot_theta) * 0.1
            self.robot_y += cmd.linear.x * math.sin(self.robot_theta) * 0.1
            
            # Update visualization every few iterations
            if rospy.Time.now().to_sec() % 2.0 < 0.1:  # Update roughly every 2 seconds
                self.visualize_waypoints()
            
            rate.sleep()
        
        self.is_navigating = False
        self.stop_robot()
        rospy.loginfo("Navigation thread ending")
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Robot stopped")
    
    def clear_waypoints(self):
        """Clear all waypoints."""
        with self.waypoints_lock:
            self.waypoints = []
            self.current_waypoint_index = 0
            self.is_navigating = False
        
        self.stop_robot()
        self.visualize_waypoints()
        rospy.loginfo("All waypoints cleared")
    
    def run(self):
        """Main run loop."""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Keep publishing TF frames
            self.publish_tf_frames()
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass 