#!/usr/bin/env python3

import rospy
import yaml
import numpy as np
import math
import cv2
import os
import tf
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

class MapHandler:
    def __init__(self, map_file):
        rospy.loginfo(f"Loading map from: {map_file}")
        
        # Load map data
        with open(map_file, 'r') as f:
            map_data = yaml.safe_load(f)
        
        self.resolution = map_data['resolution']  # meters per pixel
        self.origin_x = map_data['origin'][0]
        self.origin_y = map_data['origin'][1]
        
        # Find the absolute path to the image file
        # If image path is relative, make it absolute based on the map.yaml location
        self.image_path = map_data['image']
        if not os.path.isabs(self.image_path):
            map_dir = os.path.dirname(os.path.abspath(map_file))
            self.image_path = os.path.join(map_dir, self.image_path)
        
        rospy.loginfo(f"Attempting to load map image from: {self.image_path}")
        
        # Load map image
        self.map_image = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
        if self.map_image is None:
            rospy.logerr(f"Failed to load map image: {self.image_path}")
            raise Exception(f"Map image not found at {self.image_path}")
        
        # In the map image, white (255) is free space, black (0) is occupied
        # Invert for easier handling (1 for obstacles, 0 for free space)
        self.occupancy_grid = np.where(self.map_image < 127, 1, 0)
        
        self.height, self.width = self.occupancy_grid.shape
        rospy.loginfo(f"Loaded map: {self.width}x{self.height} pixels, resolution: {self.resolution}m/px")

    def is_obstacle(self, x, y):
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        
        # Check if within bounds
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return self.occupancy_grid[grid_y, grid_x] == 1
        return True  # Consider out-of-bounds as obstacles

    def world_to_grid(self, x, y):
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        # Convert grid coordinates to world coordinates
        x = grid_x * self.resolution + self.origin_x
        y = grid_y * self.resolution + self.origin_y
        return x, y

class PathPlanner:
    def __init__(self, map_handler):
        self.map = map_handler
    
    def plan_path(self, start_x, start_y, goal_x, goal_y):
        # Simple A* implementation for path planning
        
        # Convert world coordinates to grid coordinates
        start_grid_x, start_grid_y = self.map.world_to_grid(start_x, start_y)
        goal_grid_x, goal_grid_y = self.map.world_to_grid(goal_x, goal_y)
        
        # Check if start or goal is in obstacle
        if self.map.is_obstacle(start_x, start_y):
            rospy.logwarn("Start position is inside an obstacle!")
            return []
        
        if self.map.is_obstacle(goal_x, goal_y):
            rospy.logwarn("Goal position is inside an obstacle!")
            return []
        
        # A* algorithm implementation
        frontier = [(0, (start_grid_x, start_grid_y))]  # (priority, (x, y))
        came_from = {}
        cost_so_far = {(start_grid_x, start_grid_y): 0}
        
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance
        
        directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]  # 8-connectivity
        
        while frontier:
            _, current = frontier.pop(0)
            
            if current == (goal_grid_x, goal_grid_y):
                break
            
            for dx, dy in directions:
                next_cell = (current[0] + dx, current[1] + dy)
                
                # Check if next_cell is valid (within bounds and not an obstacle)
                next_x, next_y = self.map.grid_to_world(next_cell[0], next_cell[1])
                if self.map.is_obstacle(next_x, next_y):
                    continue
                
                # Cost is higher for diagonal movements
                move_cost = 1.4 if abs(dx) + abs(dy) == 2 else 1.0 
                new_cost = cost_so_far[current] + move_cost
                
                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    priority = new_cost + heuristic(next_cell, (goal_grid_x, goal_grid_y))
                    
                    # Insert into frontier with priority
                    i = 0
                    while i < len(frontier) and frontier[i][0] < priority:
                        i += 1
                    frontier.insert(i, (priority, next_cell))
                    
                    came_from[next_cell] = current
        
        # Reconstruct path
        if (goal_grid_x, goal_grid_y) not in came_from and (goal_grid_x, goal_grid_y) != (start_grid_x, start_grid_y):
            rospy.logwarn("No path found!")
            return []
        
        current = (goal_grid_x, goal_grid_y)
        path = []
        
        while current != (start_grid_x, start_grid_y):
            world_x, world_y = self.map.grid_to_world(current[0], current[1])
            path.append((world_x, world_y))
            if current not in came_from:
                break
            current = came_from[current]
        
        world_x, world_y = self.map.grid_to_world(start_grid_x, start_grid_y)
        path.append((world_x, world_y))
        path.reverse()  # Reverse to get start-to-goal order
        
        # Simplify path by removing unnecessary waypoints
        simplified_path = self.simplify_path(path)
        return simplified_path
    
    def simplify_path(self, path):
        # Simple path simplification to reduce waypoints
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        for i in range(1, len(path) - 1):
            # Check if we can skip this waypoint by going directly from the previous to the next
            prev_x, prev_y = simplified[-1]
            curr_x, curr_y = path[i]
            next_x, next_y = path[i + 1]
            
            # Skip if the angle change is small
            angle1 = math.atan2(curr_y - prev_y, curr_x - prev_x)
            angle2 = math.atan2(next_y - curr_y, next_x - curr_x)
            
            if abs(angle1 - angle2) > 0.3:  # If angle change is significant
                simplified.append((curr_x, curr_y))
        
        simplified.append(path[-1])
        return simplified

class RobotController:
    def __init__(self):
        rospy.init_node('autonomous_navigation', anonymous=True)
        
        # Robot state
        self.position = None
        self.orientation = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.2  # m
        self.orientation_tolerance = 0.1  # rad
        
        # Load map
        map_file = rospy.get_param('~map_file', '')
        if not map_file:
            # Try to get absolute path
            package_path = rospy.get_param('~package_path', '')
            if package_path:
                map_file = os.path.join(package_path, 'maps/my_map.yaml')
            else:
                # Fallback to the current directory + relative path
                script_dir = os.path.dirname(os.path.abspath(__file__))
                package_dir = os.path.dirname(os.path.dirname(script_dir))  # Go up two levels
                map_file = os.path.join(package_dir, 'maps/my_map.yaml')
        
        rospy.loginfo(f"Using map file: {map_file}")
        self.map_handler = MapHandler(map_file)
        self.path_planner = PathPlanner(self.map_handler)
        
        # Path tracking
        self.current_path = []
        self.current_waypoint_index = 0
        self.is_executing_path = False
        
        # TF listener for robot position
        self.tf_listener = tf.TransformListener()
        
        # Wait for the TF tree to be properly connected
        self.wait_for_tf_connection()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        self.current_goal_pub = rospy.Publisher('/current_goal', Marker, queue_size=1)
        
        # Debug publisher - publish a test odometry message if none is received
        self.debug_odom_pub = rospy.Publisher('/odom_debug', Odometry, queue_size=10)
        
        # Check what odometry topics are available
        available_topics = []
        try:
            available_topics = [topic for topic, _ in rospy.get_published_topics() if 'odom' in topic.lower()]
            rospy.loginfo(f"Available odometry topics: {available_topics}")
        except Exception as e:
            rospy.logwarn(f"Failed to get published topics: {e}")
        
        # Try multiple possible odometry topics
        self.odom_topics = [
            '/odom',                         # Standard ROS naming
            '/tracer/odom',                  # Namespaced
            '/tracer_controller/odom',       # Controller specific
            '/tracer_velocity_controller/odom',
            '/gazebo/odom',
            '/odometry/filtered'             # If using robot_localization
        ]
        
        # Add any discovered topics that have 'odom' in them
        self.odom_topics.extend([t for t in available_topics if t not in self.odom_topics])
        
        # Subscribe to all possible topics
        self.odom_subscribers = []
        for topic in self.odom_topics:
            rospy.loginfo(f"Subscribing to odometry topic: {topic}")
            self.odom_subscribers.append(rospy.Subscriber(topic, Odometry, self.odom_callback))
        
        # Subscribe to goal from RViz
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Also subscribe to odometry as a fallback
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Fallback - after a timeout, create a fake position if no real odometry received
        self.odom_timeout = rospy.Timer(rospy.Duration(30), self.odom_timeout_callback, oneshot=True)
        
        rospy.loginfo("Autonomous navigation initialized")
    
    def wait_for_tf_connection(self):
        """Wait for the TF tree to be properly connected."""
        rospy.loginfo("Waiting for TF connection between map and base_link...")
        
        # We're generous here - wait up to 60 seconds
        start_time = rospy.Time.now()
        max_wait = rospy.Duration(60.0)
        
        # Check every second
        rate = rospy.Rate(1.0)
        
        while not rospy.is_shutdown():
            try:
                # Try to get the transform
                self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                rospy.loginfo("Transform between map and base_link is now available")
                return True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # Check if we've waited too long
                if (rospy.Time.now() - start_time) > max_wait:
                    rospy.logwarn(f"Timed out waiting for transform: {e}")
                    rospy.logwarn("Proceeding anyway, but navigation may not work correctly")
                    return False
                
                # Try falling back to odometry position
                if self.position is None:
                    self.get_robot_pose_from_tf()
                
                # Publishing our own transform as a fallback
                try:
                    self.tf_listener.getFrameStrings()
                    frames = self.tf_listener.getFrameStrings()
                    rospy.loginfo(f"Available TF frames: {frames}")
                    
                    # If we have odom and base_link, but no map, add the map->odom transform
                    if 'odom' in frames and 'base_link' in frames and 'map' not in frames:
                        rospy.loginfo("Publishing fallback map->odom transform")
                        br = tf.TransformBroadcaster()
                        br.sendTransform((0, 0, 0),
                                        (0, 0, 0, 1),
                                        rospy.Time.now(),
                                        "odom",
                                        "map")
                except Exception as e:
                    rospy.logwarn(f"Error checking TF frames: {e}")
                
                rospy.loginfo("Waiting for transform between map and base_link...")
                rate.sleep()
    
    def get_robot_pose_from_tf(self):
        """Get robot pose from TF transform."""
        try:
            # First try the map->base_link transform directly
            if self.tf_listener.frameExists("map") and self.tf_listener.frameExists("base_link"):
                self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(0.5))
                (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                self.position = (trans[0], trans[1])
                _, _, self.orientation = euler_from_quaternion(rot)
                return True
            
            # If that fails, try odom->base_link and assume map->odom is identity
            elif self.tf_listener.frameExists("odom") and self.tf_listener.frameExists("base_link"):
                self.tf_listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(0.5))
                (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
                self.position = (trans[0], trans[1])
                _, _, self.orientation = euler_from_quaternion(rot)
                return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"TF Error: {e}")
        
        return False
    
    def odom_callback(self, msg):
        """Odometry callback as fallback for TF."""
        # Only use odom if we don't have a TF-based position
        if self.position is None:
            self.position = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            )
            
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            self.orientation = yaw
            
            rospy.loginfo_once("Using odometry for positioning")
    
    def odom_timeout_callback(self, event):
        rospy.logwarn("No odometry data received after timeout! Setting a default position.")
        # Set a default position to allow navigation
        self.position = (0.0, 0.0)
        self.orientation = 0.0
        
        # Publish a fake odometry message for debugging
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        self.debug_odom_pub.publish(odom)
        
        rospy.loginfo("Set default position, proceeding with navigation")
        
        # Run the demo path
        self.plan_and_execute(2.0, 2.0)
    
    def goal_callback(self, msg):
        """Handle goal from RViz."""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        # Get the current orientation from the quaternion
        orientation_q = msg.pose.orientation
        _, _, target_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        rospy.loginfo(f"Received goal: position=({goal_x}, {goal_y}), orientation={target_yaw}")
        
        # Try to update position using TF
        if not self.get_robot_pose_from_tf() and self.position is None:
            rospy.logwarn("Cannot plan path: robot position unknown")
            return
        
        self.plan_and_execute(goal_x, goal_y, target_yaw)
    
    def plan_and_execute(self, goal_x, goal_y, target_orientation=None):
        """Plan and execute a path to the goal."""
        if self.position is None:
            rospy.logwarn("Cannot plan path: robot position unknown")
            return
        
        start_x, start_y = self.position
        
        rospy.loginfo(f"Planning path from ({start_x}, {start_y}) to ({goal_x}, {goal_y})")
        
        # Plan path
        path = self.path_planner.plan_path(start_x, start_y, goal_x, goal_y)
        
        if not path:
            # If planning fails, just try to go directly to the goal
            rospy.logwarn("Failed to plan a detailed path, trying direct path to goal")
            path = [(start_x, start_y), (goal_x, goal_y)]
        
        rospy.loginfo(f"Path planned with {len(path)} waypoints")
        self.current_path = path
        self.current_waypoint_index = 0
        self.is_executing_path = True
        self.target_orientation = target_orientation
        
        # Visualize path
        self.visualize_path(path)
        
        # Execute path in a separate thread to avoid blocking
        rospy.Timer(rospy.Duration(0.1), self.execute_path_callback, oneshot=False)
    
    def execute_path_callback(self, event):
        """Execute current path (called by timer)."""
        if self.is_executing_path:
            # Update robot position from TF
            self.get_robot_pose_from_tf()
            
            if self.position is None:
                rospy.logwarn_throttle(1, "Cannot execute path: robot position unknown")
                return
            
            self.execute_path()
    
    def execute_path(self):
        """Execute the current path."""
        if not self.current_path or self.current_waypoint_index >= len(self.current_path):
            if self.is_executing_path:
                rospy.loginfo("Path execution completed")
                self.stop_robot()
                self.is_executing_path = False
            return
        
        # Get current target waypoint
        target_x, target_y = self.current_path[self.current_waypoint_index]
        self.visualize_current_goal(target_x, target_y)
        
        # Calculate distance and angle to target
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # If we've reached the waypoint, move to the next one
        if distance < self.position_tolerance:
            rospy.loginfo(f"Reached waypoint {self.current_waypoint_index+1}/{len(self.current_path)}")
            self.current_waypoint_index += 1
            
            # If this was the final waypoint and we have a target orientation, rotate to it
            if self.current_waypoint_index >= len(self.current_path) and self.target_orientation is not None:
                self.rotate_to_orientation(self.target_orientation)
                self.target_orientation = None  # Clear it so we don't keep trying
            
            return
        
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.orientation
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create and publish Twist message
        cmd = Twist()
        
        # If we need to rotate significantly, do that first
        if abs(angle_diff) > self.orientation_tolerance:
            # Only rotate, don't move forward
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            rospy.loginfo_throttle(2, f"Rotating to face waypoint: angle_diff={angle_diff:.2f}")
        else:
            # We're facing the right direction, move forward
            cmd.linear.x = self.linear_speed
            # Keep adjusting orientation as we move
            cmd.angular.z = 0.5 * angle_diff  # proportional control
            rospy.loginfo_throttle(2, f"Moving to waypoint: distance={distance:.2f}, angle_diff={angle_diff:.2f}")
        
        self.cmd_vel_pub.publish(cmd)
    
    def rotate_to_orientation(self, target_orientation):
        """Rotate the robot to face a specific orientation."""
        if self.orientation is None:
            return False
        
        # Calculate angle difference
        angle_diff = target_orientation - self.orientation
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # If we're already at the target orientation, we're done
        if abs(angle_diff) < self.orientation_tolerance:
            return True
        
        # Create and publish Twist message
        cmd = Twist()
        cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        self.cmd_vel_pub.publish(cmd)
        
        rospy.loginfo(f"Rotating to target orientation: angle_diff={angle_diff:.2f}")
        
        # Give time for rotation
        rospy.sleep(abs(angle_diff) / self.angular_speed)
        
        # Stop rotation
        self.stop_robot()
        return True
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def visualize_path(self, path):
        """Visualize the planned path in RViz."""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = rospy.Time.now()
        
        for x, y in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def visualize_current_goal(self, x, y):
        """Visualize the current goal in RViz."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'current_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.current_goal_pub.publish(marker)
    
    def run(self):
        """Main run loop."""
        rospy.loginfo("Starting autonomous navigation")
        
        # Wait for things to initialize
        rospy.sleep(1.0)
        
        # Try to get initial robot position
        if not self.get_robot_pose_from_tf() and self.position is None:
            rospy.logwarn("Cannot get initial robot position. Using (0,0) as default.")
            self.position = (0.0, 0.0)
            self.orientation = 0.0
        
        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Ready to receive navigation goals from RViz")
        rospy.loginfo("Use the '2D Nav Goal' button in RViz to set goals")
        
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except Exception as e:
        rospy.logerr(f"Autonomous navigation failed: {str(e)}")
        # Print full stack trace
        import traceback
        rospy.logerr(traceback.format_exc())
    except rospy.ROSInterruptException:
        pass 