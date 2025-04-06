#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, TransformStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
# Force matplotlib to use a non-interactive backend
import matplotlib
matplotlib.use('Agg')  # Use the 'Agg' backend which doesn't require a GUI
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import math
from std_msgs.msg import ColorRGBA, Float64, Bool
import os
from datetime import datetime
import tf2_ros
from tf2_ros import TransformException

class EnhancedVisualizer(Node):
    def __init__(self):
        super().__init__('enhanced_tag_visualizer')
        
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to AprilTag detections
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.tag_callback,
            10)
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/apriltag/visualization_markers',
            10)
            
        # Publisher for distance as a ROS topic
        self.distance_pub = self.create_publisher(
            Float64,
            '/apriltag/tag0_center_distance',
            10)
            
        # Publisher for tag0 inside box status
        self.inside_pub = self.create_publisher(
            Bool,
            '/apriltag/tag0_inside_box',
            10)
        
        # Initialize tag positions
        self.tag0_pos = None
        self.tag1_pos = None
        self.tag2_pos = None
        self.center_pos = None
        
        # Initialize distance data
        self.timestamps = []
        self.distances = []
        self.inside_status = []
        self.max_data_points = 100
        
        # Create a timer for publishing markers
        self.marker_timer = self.create_timer(0.1, self.publish_markers)
        
        # Create a timer for saving the plot periodically
        self.plot_timer = self.create_timer(2.0, self.save_plot)
        
        # Create a timer for checking tag positions from TF
        self.tf_timer = self.create_timer(0.1, self.update_tag_positions)
        
        # Setup the initial plot
        self.setup_plot()
        
    def setup_plot(self):
        # Create figure with two subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 10))
        
        # Top plot: Distance vs Time
        self.time_line, = self.ax1.plot([], [], 'r-', lw=2)
        self.ax1.set_title('Distance from Tag 0 to Center Point Over Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Distance (m)')
        self.ax1.grid(True)
        
        # Bottom plot: 2D Position
        self.ax2.set_title('Tag Positions (2D Top-Down View)')
        self.ax2.set_xlabel('X (m)')
        self.ax2.set_ylabel('Y (m)')
        self.ax2.grid(True)
        
        # Initialize position plots
        self.tag0_point, = self.ax2.plot([], [], 'ro', markersize=10, label='Tag 0')
        self.tag1_point, = self.ax2.plot([], [], 'go', markersize=10, label='Tag 1')
        self.tag2_point, = self.ax2.plot([], [], 'bo', markersize=10, label='Tag 2')
        self.center_point, = self.ax2.plot([], [], 'mo', markersize=8, label='Center')
        self.distance_line, = self.ax2.plot([], [], 'k--', lw=1)
        self.ax2.legend()
        
        # Create output directory if it doesn't exist
        os.makedirs('/tmp/apriltag_plots', exist_ok=True)
    
    def update_tag_positions(self):
        """Get tag positions from TF frames"""
        try:
            # Look up transforms for each tag
            tag0_tf = self.tf_buffer.lookup_transform('default_cam', 'tag_0', rclpy.time.Time())
            tag1_tf = self.tf_buffer.lookup_transform('default_cam', 'tag_1', rclpy.time.Time())
            tag2_tf = self.tf_buffer.lookup_transform('default_cam', 'tag_2', rclpy.time.Time())
            
            # Store positions as Point objects
            self.tag0_pos = Point()
            self.tag0_pos.x = tag0_tf.transform.translation.x
            self.tag0_pos.y = tag0_tf.transform.translation.y
            self.tag0_pos.z = tag0_tf.transform.translation.z
            
            # Calculate average height of tag1 and tag2
            tag1_tag2_avg_height = (tag1_tf.transform.translation.z + tag2_tf.transform.translation.z) / 2
            
            # Add height offset to tag0 to match tag1 and tag2 average height
            height_offset = tag1_tag2_avg_height - self.tag0_pos.z
            self.tag0_pos.z += height_offset
            
            self.tag1_pos = Point()
            self.tag1_pos.x = tag1_tf.transform.translation.x
            self.tag1_pos.y = tag1_tf.transform.translation.y
            self.tag1_pos.z = tag1_tf.transform.translation.z
            
            self.tag2_pos = Point()
            self.tag2_pos.x = tag2_tf.transform.translation.x
            self.tag2_pos.y = tag2_tf.transform.translation.y
            self.tag2_pos.z = tag2_tf.transform.translation.z
            
            # Calculate center position
            self.calculate_center_and_distance()
            
            # Log the height adjustment
            self.get_logger().debug(f'Tag0 height adjusted by {height_offset:.3f}m to match tag1-tag2 average height')
            
        except TransformException as ex:
            self.get_logger().debug(f'Could not transform: {ex}')
            pass
    
    def tag_callback(self, msg):
        # This is now only used to detect when new tag data is available
        # Actual position data comes from the TF frames
        if msg.detections:
            self.get_logger().debug(f'Received {len(msg.detections)} tag detections')
    
    def calculate_center_and_distance(self):
        if self.tag0_pos and self.tag1_pos and self.tag2_pos:
            # Calculate center point of diagonal between tag1 and tag2
            center_x = (self.tag1_pos.x + self.tag2_pos.x) / 2
            center_y = (self.tag1_pos.y + self.tag2_pos.y) / 2
            center_z = (self.tag1_pos.z + self.tag2_pos.z) / 2
            
            # Store center position
            self.center_pos = Point()
            self.center_pos.x = center_x
            self.center_pos.y = center_y
            self.center_pos.z = center_z
            
            # Calculate 2D distance (x-y plane) from tag0 to center
            distance_2d = math.sqrt(
                (self.tag0_pos.x - center_x)**2 + 
                (self.tag0_pos.y - center_y)**2
            )
            
            # Check if tag0 is inside the box
            is_inside = self.is_tag0_inside_box()
            
            # Store data
            self.timestamps.append(self.get_clock().now().seconds_nanoseconds()[0])
            self.distances.append(distance_2d)
            self.inside_status.append(is_inside)
            
            # Keep only the latest max_data_points
            if len(self.timestamps) > self.max_data_points:
                self.timestamps = self.timestamps[-self.max_data_points:]
                self.distances = self.distances[-self.max_data_points:]
                self.inside_status = self.inside_status[-self.max_data_points:]
            
            # Log the distance and inside/outside status
            status = "INSIDE" if is_inside else "OUTSIDE"
            self.get_logger().info(f'2D Distance: {distance_2d:.3f} m, Tag 0 is {status} the box')
            
            # Publish distance and inside status
            dist_msg = Float64()
            dist_msg.data = distance_2d
            self.distance_pub.publish(dist_msg)
            
            inside_msg = Bool()
            inside_msg.data = is_inside
            self.inside_pub.publish(inside_msg)
            
            return distance_2d
        return None
    
    def is_tag0_inside_box(self):
        """Check if tag0 is inside the box formed by tag1 and tag2 as opposite corners"""
        if not (self.tag0_pos and self.tag1_pos and self.tag2_pos):
            return False
            
        # Get box boundaries from tag1 and tag2
        min_x = min(self.tag1_pos.x, self.tag2_pos.x)
        max_x = max(self.tag1_pos.x, self.tag2_pos.x)
        min_y = min(self.tag1_pos.y, self.tag2_pos.y)
        max_y = max(self.tag1_pos.y, self.tag2_pos.y)
        min_z = min(self.tag1_pos.z, self.tag2_pos.z)
        max_z = max(self.tag1_pos.z, self.tag2_pos.z)
        
        # Check if tag0 is inside the box (in 3D)
        return (min_x <= self.tag0_pos.x <= max_x and 
                min_y <= self.tag0_pos.y <= max_y and
                min_z <= self.tag0_pos.z <= max_z)
    
    def save_plot(self):
        if len(self.timestamps) > 1 and self.tag0_pos and self.tag1_pos and self.tag2_pos and self.center_pos:
            # Update time series plot
            t0 = self.timestamps[0]
            rel_times = [t - t0 for t in self.timestamps]
            
            # Update the time plot data
            self.time_line.set_data(rel_times, self.distances)
            
            # Adjust the plot limits
            self.ax1.set_xlim(min(rel_times), max(rel_times))
            if self.distances:
                min_dist = min(self.distances)
                max_dist = max(self.distances)
                range_dist = max(0.1, max_dist - min_dist)
                self.ax1.set_ylim(max(0, min_dist - 0.1 * range_dist), 
                                max_dist + 0.1 * range_dist)
            
            # Update position plot
            # Update tag positions
            self.tag0_point.set_data([self.tag0_pos.x], [self.tag0_pos.y])
            self.tag1_point.set_data([self.tag1_pos.x], [self.tag1_pos.y])
            self.tag2_point.set_data([self.tag2_pos.x], [self.tag2_pos.y])
            self.center_point.set_data([self.center_pos.x], [self.center_pos.y])
            
            # Determine box corners
            min_x = min(self.tag1_pos.x, self.tag2_pos.x)
            max_x = max(self.tag1_pos.x, self.tag2_pos.x)
            min_y = min(self.tag1_pos.y, self.tag2_pos.y)
            max_y = max(self.tag1_pos.y, self.tag2_pos.y)
            
            # Draw box
            box_x = [min_x, max_x, max_x, min_x, min_x]
            box_y = [min_y, min_y, max_y, max_y, min_y]
            
            # Check if we already have a box plot
            if not hasattr(self, 'box_plot'):
                self.box_plot, = self.ax2.plot(box_x, box_y, 'g-', lw=2, label='Box')
                self.ax2.legend()
            else:
                self.box_plot.set_data(box_x, box_y)
            
            # Set box color based on if tag0 is inside
            is_inside = self.is_tag0_inside_box()
            if is_inside:
                self.box_plot.set_color('green')
            else:
                self.box_plot.set_color('red')
            
            # Update the distance line
            self.distance_line.set_data(
                [self.tag0_pos.x, self.center_pos.x],
                [self.tag0_pos.y, self.center_pos.y]
            )
            
            # Auto-adjust the position plot limits
            x_vals = [self.tag0_pos.x, self.tag1_pos.x, self.tag2_pos.x, self.center_pos.x]
            y_vals = [self.tag0_pos.y, self.tag1_pos.y, self.tag2_pos.y, self.center_pos.y]
            
            x_min, x_max = min(x_vals), max(x_vals)
            y_min, y_max = min(y_vals), max(y_vals)
            
            # Add some padding
            x_range = max(0.1, x_max - x_min)
            y_range = max(0.1, y_max - y_min)
            
            self.ax2.set_xlim(x_min - 0.2 * x_range, x_max + 0.2 * x_range)
            self.ax2.set_ylim(y_min - 0.2 * y_range, y_max + 0.2 * y_range)
            
            # Save the plot
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plt.savefig(f'/tmp/apriltag_plots/tag_visualization_{timestamp}.png')
            self.get_logger().info(f'Plot saved to /tmp/apriltag_plots/tag_visualization_{timestamp}.png')
    
    def publish_markers(self):
        if self.tag0_pos and self.tag1_pos and self.tag2_pos and self.center_pos:
            marker_array = MarkerArray()
            
            # Create square box marker from tag1 and tag2 as opposite corners
            box_marker = Marker()
            box_marker.header.frame_id = "default_cam"
            box_marker.header.stamp = self.get_clock().now().to_msg()
            box_marker.ns = "box"
            box_marker.id = 0
            box_marker.type = Marker.LINE_STRIP
            box_marker.action = Marker.ADD
            box_marker.pose.orientation.w = 1.0
            box_marker.scale.x = 0.01  # Line width
            
            # Determine box corners
            min_x = min(self.tag1_pos.x, self.tag2_pos.x)
            max_x = max(self.tag1_pos.x, self.tag2_pos.x)
            min_y = min(self.tag1_pos.y, self.tag2_pos.y)
            max_y = max(self.tag1_pos.y, self.tag2_pos.y)
            min_z = min(self.tag1_pos.z, self.tag2_pos.z)
            max_z = max(self.tag1_pos.z, self.tag2_pos.z)
            
            # Create the corners of the box (3D box with all corners)
            corners = [
                # Bottom face
                (min_x, min_y, min_z),
                (max_x, min_y, min_z),
                (max_x, max_y, min_z),
                (min_x, max_y, min_z),
                (min_x, min_y, min_z),
                
                # Connect to top face
                (min_x, min_y, max_z),
                
                # Top face
                (max_x, min_y, max_z),
                (max_x, max_y, max_z),
                (min_x, max_y, max_z),
                (min_x, min_y, max_z),
                
                # Connect remaining edges
                (min_x, min_y, min_z),  # Already connected bottom-front-left to top-front-left
                (min_x, max_y, min_z),
                (min_x, max_y, max_z),
                (min_x, max_y, min_z),  # Connect bottom-back-left to top-back-left
                (max_x, max_y, min_z),
                (max_x, max_y, max_z),
                (max_x, max_y, min_z),  # Connect bottom-back-right to top-back-right
                (max_x, min_y, min_z),
                (max_x, min_y, max_z)
            ]
            
            # Add points for the box
            for x, y, z in corners:
                p = Point()
                p.x = x
                p.y = y
                p.z = z
                box_marker.points.append(p)
            
            # Check if tag0 is inside the box
            is_inside = self.is_tag0_inside_box()
            
            # Set box color - green if tag0 is inside, red if outside
            color = ColorRGBA()
            if is_inside:
                color.r = 0.0
                color.g = 1.0
                color.b = 0.0
            else:
                color.r = 1.0
                color.g = 0.0
                color.b = 0.0
            color.a = 1.0
            box_marker.color = color
            
            marker_array.markers.append(box_marker)
            
            # Add tag0 marker
            tag0_marker = Marker()
            tag0_marker.header.frame_id = "default_cam"
            tag0_marker.header.stamp = self.get_clock().now().to_msg()
            tag0_marker.ns = "tag_positions"
            tag0_marker.id = 0
            tag0_marker.type = Marker.SPHERE
            tag0_marker.action = Marker.ADD
            tag0_marker.pose.position.x = self.tag0_pos.x
            tag0_marker.pose.position.y = self.tag0_pos.y
            tag0_marker.pose.position.z = self.tag0_pos.z
            tag0_marker.pose.orientation.w = 1.0
            tag0_marker.scale.x = 0.05
            tag0_marker.scale.y = 0.05
            tag0_marker.scale.z = 0.05
            
            # Set color for tag0 - blue
            color = ColorRGBA()
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0
            color.a = 1.0
            tag0_marker.color = color
            
            marker_array.markers.append(tag0_marker)
            
            # Publish the marker array
            self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = EnhancedVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 