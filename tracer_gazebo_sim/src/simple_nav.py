#!/usr/bin/env python3

import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

class SimpleNavigation:
    def __init__(self):
        rospy.init_node('simple_navigation')
        
        # Initialize publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('/current_goal', Marker, queue_size=1)
        
        # Subscribe to RViz goals
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Initialize TF listener
        self.tf_listener = tf.TransformListener()
        
        # Current goal
        self.current_goal = None
        
        # Wait a bit for things to initialize
        rospy.sleep(2.0)
        
        # Log status
        rospy.loginfo("Simple navigation initialized")
        
        # Try to get available frames
        try:
            frames = self.tf_listener.getFrameStrings()
            rospy.loginfo("Available TF frames: %s", frames)
        except Exception as e:
            rospy.logwarn("Error getting TF frames: %s", e)
    
    def goal_callback(self, msg):
        """Handle goals from RViz."""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        # Get the current orientation from the quaternion
        orientation_q = msg.pose.orientation
        _, _, target_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        rospy.loginfo("Received goal: position=(%f, %f), orientation=%f", goal_x, goal_y, target_yaw)
        
        # Set current goal
        self.current_goal = (goal_x, goal_y, target_yaw)
        
        # Visualize the goal
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'goals'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.goal_pub.publish(marker)
        
        # Start moving to the goal
        self.move_to_goal()
    
    def get_robot_pose(self):
        """Get the robot's current pose."""
        try:
            # Try map->base_link transform
            if self.tf_listener.frameExists("map") and self.tf_listener.frameExists("base_link"):
                try:
                    self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                    (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                    _, _, yaw = euler_from_quaternion(rot)
                    return trans[0], trans[1], yaw
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn("Error looking up transform: %s", e)
            
            # As fallback, try odom->base_link transform
            if self.tf_listener.frameExists("odom") and self.tf_listener.frameExists("base_link"):
                try:
                    self.tf_listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                    (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
                    _, _, yaw = euler_from_quaternion(rot)
                    return trans[0], trans[1], yaw
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn("Error looking up transform: %s", e)
        except Exception as e:
            rospy.logwarn("Error in get_robot_pose: %s", e)
        
        return None, None, None
    
    def move_to_goal(self):
        """Move the robot toward the current goal."""
        if self.current_goal is None:
            return
        
        goal_x, goal_y, goal_yaw = self.current_goal
        
        # Control parameters
        linear_speed = 0.3  # m/s
        angular_speed = 0.5  # rad/s
        position_tolerance = 0.2  # m
        orientation_tolerance = 0.1  # rad
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown() and self.current_goal is not None:
            # Get robot's current pose
            x, y, yaw = self.get_robot_pose()
            
            if x is None:
                rospy.logwarn_throttle(1, "Could not get robot pose")
                rate.sleep()
                continue
            
            # Calculate distance and angle to goal
            dx = goal_x - x
            dy = goal_y - y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # If we've reached the goal position
            if distance < position_tolerance:
                rospy.loginfo("Reached goal position")
                
                # Turn to goal orientation
                angle_diff = goal_yaw - yaw
                # Normalize angle to [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                if abs(angle_diff) < orientation_tolerance:
                    rospy.loginfo("Reached goal orientation, navigation complete")
                    self.stop_robot()
                    self.current_goal = None
                    break
                
                # Just rotate to face goal orientation
                cmd = Twist()
                cmd.angular.z = angular_speed if angle_diff > 0 else -angular_speed
                self.cmd_vel_pub.publish(cmd)
            else:
                # Calculate target angle to goal
                target_angle = math.atan2(dy, dx)
                
                # Calculate angle difference
                angle_diff = target_angle - yaw
                # Normalize angle to [-pi, pi]
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                cmd = Twist()
                
                # If we need to turn significantly, do that first
                if abs(angle_diff) > orientation_tolerance:
                    cmd.angular.z = angular_speed if angle_diff > 0 else -angular_speed
                else:
                    # We're facing the right direction, move forward
                    cmd.linear.x = linear_speed
                    # Keep adjusting angle as we move
                    cmd.angular.z = 0.5 * angle_diff
                
                self.cmd_vel_pub.publish(cmd)
                
                rospy.loginfo_throttle(1, "Moving to goal: distance=%.2f, angle_diff=%.2f", distance, angle_diff)
            
            rate.sleep()
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Robot stopped")
    
    def run(self):
        """Main run loop."""
        rospy.loginfo("Simple navigation ready for goals")
        
        # Publish transforms periodically to ensure they're established
        br = tf.TransformBroadcaster()
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Republish map->odom transform
            br.sendTransform((0, 0, 0),
                            (0, 0, 0, 1),
                            rospy.Time.now(),
                            "odom",
                            "map")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        navigator = SimpleNavigation()
        navigator.run()
    except Exception as e:
        rospy.logerr("Navigation failed: %s", e)
        import traceback
        rospy.logerr(traceback.format_exc())
    except rospy.ROSInterruptException:
        pass 