#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import tf
import threading
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from std_msgs.msg import ColorRGBA

class ColorNavigator:
    def __init__(self):
        rospy.init_node('color_navigator')
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.markers_pub = rospy.Publisher('/color_markers', MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher('/color_path', Path, queue_size=1)
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.2  # m
        
        # Initialize the robot position
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0
        
        # List of color waypoints (x, y, theta, color_name)
        self.waypoints = []
        
        # Current waypoint index
        self.current_waypoint_index = 0
        
        # Flag to indicate if we're currently navigating
        self.is_navigating = False
        
        # Lock for thread-safe access
        self.lock = threading.Lock()
        
        # Load map parameters from ROS parameter server or defaults
        map_file = rospy.get_param('~map_file', 'maps/my_map.yaml')
        
        # Load the map and detect colors
        self.load_map(map_file)
        self.detect_colors()
        
        # Start publishing TF frames
        self.publish_tf_frames()
        
        # Start the visualization
        self.visualize_color_waypoints()
        
        rospy.loginfo("Color Navigator initialized")
        rospy.loginfo(f"Detected {len(self.waypoints)} colored areas in the map")
        
        # Start navigation after a brief delay
        rospy.Timer(rospy.Duration(3.0), self.start_navigation, oneshot=True)
    
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
    
    def load_map(self, map_file):
        """Load map from file."""
        import yaml
        import os
        
        rospy.loginfo(f"Loading map file: {map_file}")
        
        # Load map data
        with open(map_file, 'r') as f:
            map_data = yaml.safe_load(f)
        
        self.resolution = map_data.get('resolution', 0.05)  # meters per pixel
        self.origin_x = map_data.get('origin', [0, 0, 0])[0]
        self.origin_y = map_data.get('origin', [0, 0, 0])[1]
        
        # Get image path
        image_path = map_data.get('image', '')
        if not os.path.isabs(image_path):
            # If path is relative, make it absolute based on map file location
            dir_path = os.path.dirname(os.path.abspath(map_file))
            image_path = os.path.join(dir_path, image_path)
        
        rospy.loginfo(f"Loading map image: {image_path}")
        
        # Load map image
        self.map_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if self.map_image is None:
            rospy.logerr(f"Failed to load map image: {image_path}")
            raise Exception(f"Map image not found at {image_path}")
        
        self.height, self.width, _ = self.map_image.shape
        rospy.loginfo(f"Map dimensions: {self.width}x{self.height} pixels, resolution: {self.resolution}m/px")
    
    def detect_colors(self):
        """Detect different colored areas in the map and create waypoints."""
        # Convert to HSV for better color detection
        hsv_image = cv2.cvtColor(self.map_image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges in HSV
        color_ranges = {
            'red': (np.array([0, 120, 70]), np.array([10, 255, 255])),  # Red
            'blue': (np.array([100, 100, 100]), np.array([140, 255, 255])),  # Blue
            'green': (np.array([40, 100, 100]), np.array([80, 255, 255])),  # Green
            'yellow': (np.array([20, 100, 100]), np.array([40, 255, 255])),  # Yellow
            'purple': (np.array([140, 100, 100]), np.array([170, 255, 255]))  # Purple
        }
        
        # Also add another red range (red wraps around the hue spectrum)
        color_ranges['red2'] = (np.array([170, 120, 70]), np.array([180, 255, 255]))
        
        # Show debug image
        cv2.imwrite('/tmp/original_map.png', self.map_image)
        
        for color_name, (lower, upper) in color_ranges.items():
            # Create mask for this color
            mask = cv2.inRange(hsv_image, lower, upper)
            
            # Add red2 to red if processing red
            if color_name == 'red':
                mask2 = cv2.inRange(hsv_image, color_ranges['red2'][0], color_ranges['red2'][1])
                mask = cv2.bitwise_or(mask, mask2)
            
            # Skip red2 as it's already processed with red
            if color_name == 'red2':
                continue
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Debug output
            cv2.imwrite(f'/tmp/{color_name}_mask.png', mask)
            
            # Process each contour to find centroids
            for i, contour in enumerate(contours):
                # Filter small contours (noise)
                if cv2.contourArea(contour) < 100:
                    continue
                
                # Calculate the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue
                
                # Get centroid coordinates in pixels
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Convert pixel coordinates to world coordinates
                world_x = cX * self.resolution + self.origin_x
                world_y = cY * self.resolution + self.origin_y
                
                # Add waypoint with default orientation
                self.waypoints.append((world_x, world_y, 0.0, color_name))
                
                rospy.loginfo(f"Found {color_name} area at ({world_x:.2f}, {world_y:.2f})")
    
    def visualize_color_waypoints(self):
        """Visualize the color waypoints in RViz."""
        marker_array = MarkerArray()
        
        # Create path message
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        # Color mapping to RGBA
        color_to_rgba = {
            'red': ColorRGBA(1.0, 0.0, 0.0, 1.0),
            'green': ColorRGBA(0.0, 1.0, 0.0, 1.0),
            'blue': ColorRGBA(0.0, 0.0, 1.0, 1.0),
            'yellow': ColorRGBA(1.0, 1.0, 0.0, 1.0),
            'purple': ColorRGBA(0.7, 0.0, 0.7, 1.0)
        }
        
        # Add markers for each waypoint
        for i, (x, y, theta, color_name) in enumerate(self.waypoints):
            # Get color from mapping or default to white
            color = color_to_rgba.get(color_name, ColorRGBA(1.0, 1.0, 1.0, 1.0))
            
            # Highlight current waypoint
            if i == self.current_waypoint_index and self.is_navigating:
                # Make the current waypoint more visible
                scale = 0.4  # Larger marker
                brightness = 1.5  # Brighter (will be clamped to 1.0 for rgb)
            else:
                scale = 0.3
                brightness = 1.0
            
            # Create marker for this waypoint
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "color_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            
            # Apply brightness factor (clamped to 1.0)
            bright_color = ColorRGBA(
                min(color.r * brightness, 1.0),
                min(color.g * brightness, 1.0),
                min(color.b * brightness, 1.0),
                color.a
            )
            marker.color = bright_color
            
            marker_array.markers.append(marker)
            
            # Add text marker to show the color name
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = rospy.Time.now()
            text.ns = "color_labels"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2  # Text height
            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White text
            text.text = color_name
            
            marker_array.markers.append(text)
            
            # Add to path
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            q = tf.transformations.quaternion_from_euler(0, 0, theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)
        
        # Add current robot position to path if we're navigating
        if self.is_navigating and self.waypoints:
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
            
            # Insert at the beginning
            path.poses.insert(0, robot_pose)
        
        # Publish the markers and path
        self.markers_pub.publish(marker_array)
        self.path_pub.publish(path)
    
    def start_navigation(self, event=None):
        """Start navigating to the color waypoints."""
        if not self.waypoints:
            rospy.logwarn("No color waypoints detected to navigate to")
            return
        
        if self.is_navigating:
            rospy.loginfo("Already navigating")
            return
        
        self.is_navigating = True
        self.current_waypoint_index = 0
        
        # Start navigation in a separate thread
        threading.Thread(target=self.navigate_to_waypoints).start()
        
        rospy.loginfo("Starting navigation to color waypoints")
        rospy.loginfo(f"First target: {self.waypoints[0][3]} at ({self.waypoints[0][0]:.2f}, {self.waypoints[0][1]:.2f})")
    
    def navigate_to_waypoints(self):
        """Navigate to all color waypoints in the detected order."""
        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("=== Starting Color-Based Navigation ===")
        
        while not rospy.is_shutdown() and self.is_navigating:
            # Check if we still have waypoints to navigate to
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("=== Reached all color waypoints ===")
                self.is_navigating = False
                self.stop_robot()
                break
            
            # Get current waypoint
            current_wp = self.waypoints[self.current_waypoint_index]
            goal_x, goal_y, goal_theta, color_name = current_wp
            
            # Calculate distance to goal
            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if we've reached the waypoint
            if distance < self.position_tolerance:
                rospy.loginfo(f"Reached {color_name} waypoint!")
                
                # Stop briefly at this waypoint
                self.stop_robot()
                rospy.sleep(1.0)  # Pause at the colored area
                
                # Move to next waypoint
                self.current_waypoint_index += 1
                
                if self.current_waypoint_index < len(self.waypoints):
                    next_wp = self.waypoints[self.current_waypoint_index]
                    rospy.loginfo(f"Moving to {next_wp[3]} waypoint at ({next_wp[0]:.2f}, {next_wp[1]:.2f})")
                else:
                    rospy.loginfo("All color waypoints completed!")
                
                # Update visualization
                self.visualize_color_waypoints()
                
                # Continue to next iteration
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
                rospy.loginfo_throttle(2, f"Turning to face {color_name} waypoint: angle_diff={angle_diff:.2f}")
            else:
                # We're roughly facing the goal, move forward and adjust orientation
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.5 * angle_diff  # Proportional control
                rospy.loginfo_throttle(2, f"Moving to {color_name} waypoint: distance={distance:.2f}, angle_diff={angle_diff:.2f}")
            
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
            # Update robot position (simple integration)
            # In a real system, this would come from odometry
            self.robot_theta += cmd.angular.z * 0.1  # dt = 0.1s
            self.robot_x += cmd.linear.x * math.cos(self.robot_theta) * 0.1
            self.robot_y += cmd.linear.x * math.sin(self.robot_theta) * 0.1
            
            # Update visualization periodically
            if rospy.Time.now().to_sec() % 1.0 < 0.1:  # Update roughly every second
                self.visualize_color_waypoints()
            
            rate.sleep()
        
        self.is_navigating = False
    
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
            
            # Periodically update visualization
            if rospy.Time.now().to_sec() % 2.0 < 0.1:  # Every 2 seconds
                self.visualize_color_waypoints()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = ColorNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass 