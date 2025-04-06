#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
import matplotlib
# Don't specify a backend, let matplotlib choose automatically
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math
import threading
import tf2_ros
from tf2_ros import TransformException
import os
from datetime import datetime

class SquareDistanceTracker(Node):
    def __init__(self):
        super().__init__('square_distance_tracker')
        
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize tag positions
        self.tag0_pos = None
        self.tag1_pos = None
        self.tag2_pos = None
        self.center_pos = None
        
        # Initialize data storage for plotting
        self.timestamps = []
        self.distances = []
        self.inside_status = []
        self.max_data_points = 100
        
        # Create output directory for debug plots
        self.debug_dir = '/tmp/apriltag_2d_plots'
        os.makedirs(self.debug_dir, exist_ok=True)
        
        # Publisher for distance and inside status
        self.distance_pub = self.create_publisher(
            Float64,
            '/apriltag/tag0_center_distance_2d',
            10)
            
        self.inside_pub = self.create_publisher(
            Bool,
            '/apriltag/tag0_inside_box_2d',
            10)
        
        # Create timer for updating tag positions
        self.tf_timer = self.create_timer(0.1, self.update_tag_positions)
        
        # Create timer for updating plots
        self.plot_timer = self.create_timer(0.5, self.update_plot)
        
        # Setup the plots
        self.setup_plots()
        
    def setup_plots(self):
        # Create a figure with 2 subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 10))
        
        # Distance vs Time plot
        self.ax1.set_title('Distance from Tag 0 to Center Point Over Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Distance (m)')
        self.ax1.grid(True)
        self.distance_line, = self.ax1.plot([], [], 'r-', lw=2)
        
        # Current distance value text
        self.current_distance_text = self.ax1.text(0.05, 0.95, '', transform=self.ax1.transAxes, 
                                                  fontsize=12, fontweight='bold', 
                                                  verticalalignment='top')
        
        # 2D Position plot (top-down view)
        self.ax2.set_title('Tag Positions (2D Top-Down View)')
        self.ax2.set_xlabel('X (m)')
        self.ax2.set_ylabel('Y (m)')
        self.ax2.grid(True)
        
        # Initialize position plots
        self.tag0_point, = self.ax2.plot([], [], 'ro', markersize=10, label='Tag 0')
        self.tag1_point, = self.ax2.plot([], [], 'go', markersize=10, label='Tag 1')
        self.tag2_point, = self.ax2.plot([], [], 'bo', markersize=10, label='Tag 2')
        self.center_point, = self.ax2.plot([], [], 'mo', markersize=8, label='Center')
        self.box_line, = self.ax2.plot([], [], 'g-', lw=2, label='Square Box')
        self.distance_line_2d, = self.ax2.plot([], [], 'k--', lw=1)
        self.ax2.legend(loc='upper left')
        
        # Add status text to the position plot
        self.status_text = self.ax2.text(0.05, 0.95, '', transform=self.ax2.transAxes, 
                                         fontsize=12, fontweight='bold', 
                                         verticalalignment='top')
        
        # Adjust layout
        plt.tight_layout()
        
        # Show the plot in non-blocking mode
        plt.ion()
        plt.show(block=False)
    
    def update_tag_positions(self):
        """Get tag positions from TF frames and process 2D components"""
        try:
            # Look up transforms for each tag
            tag0_tf = self.tf_buffer.lookup_transform('default_cam', 'tag_0', rclpy.time.Time())
            tag1_tf = self.tf_buffer.lookup_transform('default_cam', 'tag_1', rclpy.time.Time())
            tag2_tf = self.tf_buffer.lookup_transform('default_cam', 'tag_2', rclpy.time.Time())
            
            # Store positions (only X and Y components)
            self.tag0_pos = Point()
            self.tag0_pos.x = tag0_tf.transform.translation.x
            self.tag0_pos.y = tag0_tf.transform.translation.y
            self.tag0_pos.z = 0.0  # Ignore Z component for 2D analysis
            
            self.tag1_pos = Point()
            self.tag1_pos.x = tag1_tf.transform.translation.x
            self.tag1_pos.y = tag1_tf.transform.translation.y
            self.tag1_pos.z = 0.0
            
            self.tag2_pos = Point()
            self.tag2_pos.x = tag2_tf.transform.translation.x
            self.tag2_pos.y = tag2_tf.transform.translation.y
            self.tag2_pos.z = 0.0
            
            # Process the positions
            self.calculate_center_and_distance()
            
        except TransformException as ex:
            self.get_logger().debug(f'Could not transform: {ex}')
            pass
    
    def calculate_center_and_distance(self):
        """Calculate center point and distance in 2D space"""
        if self.tag0_pos and self.tag1_pos and self.tag2_pos:
            # Calculate center point of diagonal between tag1 and tag2
            center_x = (self.tag1_pos.x + self.tag2_pos.x) / 2
            center_y = (self.tag1_pos.y + self.tag2_pos.y) / 2
            
            # Store center position
            self.center_pos = Point()
            self.center_pos.x = center_x
            self.center_pos.y = center_y
            self.center_pos.z = 0.0
            
            # Calculate 2D distance from tag0 to center
            distance_2d = math.sqrt(
                (self.tag0_pos.x - center_x)**2 + 
                (self.tag0_pos.y - center_y)**2
            )
            
            # Check if tag0 is inside the square box
            is_inside = self.is_tag0_inside_square()
            
            # Store data for plotting
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.timestamps.append(current_time)
            self.distances.append(distance_2d)
            self.inside_status.append(is_inside)
            
            # Keep only the latest max_data_points
            if len(self.timestamps) > self.max_data_points:
                self.timestamps = self.timestamps[-self.max_data_points:]
                self.distances = self.distances[-self.max_data_points:]
                self.inside_status = self.inside_status[-self.max_data_points:]
            
            # Log the distance and inside/outside status
            status = "INSIDE" if is_inside else "OUTSIDE"
            self.get_logger().info(f'2D Distance: {distance_2d:.3f} m, Tag 0 is {status} the square box')
            
            # Publish distance and inside status
            dist_msg = Float64()
            dist_msg.data = distance_2d
            self.distance_pub.publish(dist_msg)
            
            inside_msg = Bool()
            inside_msg.data = is_inside
            self.inside_pub.publish(inside_msg)
            
            return distance_2d
        return None
    
    def create_perfect_square(self):
        """Create a perfect square box centered at the center point of tag1 and tag2"""
        if not (self.tag1_pos and self.tag2_pos and self.center_pos):
            return None, None
            
        # Calculate the diagonal distance between tag1 and tag2
        diagonal = math.sqrt(
            (self.tag1_pos.x - self.tag2_pos.x)**2 + 
            (self.tag1_pos.y - self.tag2_pos.y)**2
        )
        
        # Use half of the diagonal as the side length of the square
        side_length = diagonal / 2
        
        # Calculate the corner points of the square centered at center_pos
        half_side = side_length / 2
        
        # Square corners
        corners_x = [
            self.center_pos.x - half_side,  # Bottom-left
            self.center_pos.x + half_side,  # Bottom-right
            self.center_pos.x + half_side,  # Top-right
            self.center_pos.x - half_side,  # Top-left
            self.center_pos.x - half_side,  # Back to Bottom-left to close the square
        ]
        
        corners_y = [
            self.center_pos.y - half_side,  # Bottom-left
            self.center_pos.y - half_side,  # Bottom-right
            self.center_pos.y + half_side,  # Top-right
            self.center_pos.y + half_side,  # Top-left
            self.center_pos.y - half_side,  # Back to Bottom-left to close the square
        ]
        
        return corners_x, corners_y
    
    def is_tag0_inside_square(self):
        """Check if tag0 is inside the perfect square box"""
        if not (self.tag0_pos and self.tag1_pos and self.tag2_pos and self.center_pos):
            return False
            
        corners_x, corners_y = self.create_perfect_square()
        if not corners_x or not corners_y:
            return False
            
        # Get square boundaries
        min_x = min(corners_x)
        max_x = max(corners_x)
        min_y = min(corners_y)
        max_y = max(corners_y)
        
        # Check if tag0 is inside the square
        return (min_x <= self.tag0_pos.x <= max_x and 
                min_y <= self.tag0_pos.y <= max_y)
    
    def update_plot(self):
        """Update the matplotlib plot with current data (called by timer)"""
        if len(self.timestamps) > 1 and self.tag0_pos and self.tag1_pos and self.tag2_pos and self.center_pos:
            # Update distance vs time plot
            t0 = self.timestamps[0]
            rel_times = [t - t0 for t in self.timestamps]
            
            # Clear axes for redraw
            self.ax1.clear()
            self.ax1.set_title('Distance from Tag 0 to Center Point Over Time')
            self.ax1.set_xlabel('Time (s)')
            self.ax1.set_ylabel('Distance (m)')
            self.ax1.grid(True)
            
            # Color the line segments based on inside/outside status
            for i in range(len(rel_times)-1):
                color = 'green' if self.inside_status[i] else 'red'
                self.ax1.plot(rel_times[i:i+2], self.distances[i:i+2], color=color, lw=2)
            
            # Update current distance text
            if self.distances:
                self.ax1.text(0.05, 0.95, f'Current Distance: {self.distances[-1]:.3f}m', 
                             transform=self.ax1.transAxes, fontsize=12, fontweight='bold',
                             verticalalignment='top')
            
            # Adjust time plot limits
            if rel_times:
                self.ax1.set_xlim(min(rel_times), max(rel_times))
                if self.distances:
                    min_dist = min(self.distances)
                    max_dist = max(self.distances)
                    range_dist = max(0.1, max_dist - min_dist)
                    self.ax1.set_ylim(max(0, min_dist - 0.1 * range_dist), 
                                    max_dist + 0.1 * range_dist)
            
            # Update position plot
            self.ax2.clear()
            self.ax2.set_title('Tag Positions and Perfect Square Box')
            self.ax2.set_xlabel('X (m)')
            self.ax2.set_ylabel('Y (m)')
            self.ax2.grid(True)
            
            # Plot tags and center
            self.ax2.plot(self.tag0_pos.x, self.tag0_pos.y, 'ro', markersize=10, label='Tag 0')
            self.ax2.plot(self.tag1_pos.x, self.tag1_pos.y, 'go', markersize=10, label='Tag 1')
            self.ax2.plot(self.tag2_pos.x, self.tag2_pos.y, 'bo', markersize=10, label='Tag 2')
            self.ax2.plot(self.center_pos.x, self.center_pos.y, 'mo', markersize=8, label='Center')
            
            # Draw distance line from tag0 to center
            self.ax2.plot([self.tag0_pos.x, self.center_pos.x], 
                         [self.tag0_pos.y, self.center_pos.y], 'k--', lw=1)
            
            # Draw perfect square
            corners_x, corners_y = self.create_perfect_square()
            if corners_x and corners_y:
                # Set square color based on if tag0 is inside
                is_inside = self.is_tag0_inside_square()
                color = 'green' if is_inside else 'red'
                self.ax2.plot(corners_x, corners_y, color=color, lw=2, label='Square Box')
                
                # Add status text
                status = "INSIDE" if is_inside else "OUTSIDE"
                self.ax2.text(0.05, 0.95, f'Tag 0 is {status} the box', 
                             transform=self.ax2.transAxes, fontsize=12, fontweight='bold',
                             verticalalignment='top', color=color)
            
            # Auto-adjust the position plot limits with padding
            x_vals = [self.tag0_pos.x, self.tag1_pos.x, self.tag2_pos.x, self.center_pos.x]
            y_vals = [self.tag0_pos.y, self.tag1_pos.y, self.tag2_pos.y, self.center_pos.y]
            
            if corners_x and corners_y:
                x_vals.extend(corners_x)
                y_vals.extend(corners_y)
            
            x_min, x_max = min(x_vals), max(x_vals)
            y_min, y_max = min(y_vals), max(y_vals)
            
            # Add some padding
            x_range = max(0.1, x_max - x_min)
            y_range = max(0.1, y_max - y_min)
            
            self.ax2.set_xlim(x_min - 0.2 * x_range, x_max + 0.2 * x_range)
            self.ax2.set_ylim(y_min - 0.2 * y_range, y_max + 0.2 * y_range)
            
            self.ax2.legend(loc='upper left')
            self.fig.tight_layout()
            
            # Draw to screen
            plt.draw()
            plt.pause(0.001)  # Small pause to allow plot to update
            
            # Save debug image every 5 seconds
            if len(self.timestamps) % 10 == 0:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{self.debug_dir}/square_tracker_{timestamp}.png"
                self.fig.savefig(filename)
                self.get_logger().info(f"Saved debug plot to {filename}")

def main():
    rclpy.init()
    node = SquareDistanceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close plot window
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 