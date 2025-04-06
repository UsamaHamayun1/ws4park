#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformListener, Buffer
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import math

class AprilTagDistanceTracker(Node):
    def __init__(self):
        super().__init__('apriltag_distance_tracker')
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to AprilTag detections
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/apriltag/detections',
            self.tag_callback,
            10)
        
        # Initialize tag positions
        self.tag0_pos = None
        self.tag1_pos = None
        self.tag2_pos = None
        
        # Initialize distance data
        self.timestamps = []
        self.distances = []
        self.max_data_points = 100
        
        # Set up the plot
        self.setup_plot()
        
    def setup_plot(self):
        # Create a figure and axis
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.line, = self.ax.plot([], [], 'r-', lw=2)
        
        # Set up plot parameters
        self.ax.set_title('Distance from Tag 0 to Center of Tag 1-2 Diagonal')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (m)')
        self.ax.grid(True)
        
        # Set up the animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=True)
        
        # Start the plot in a separate thread
        self.plot_thread = threading.Thread(target=plt.show)
        self.plot_thread.daemon = True
        self.plot_thread.start()
    
    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id == 0:
                self.tag0_pos = detection.pose.pose.pose.position
            elif detection.id == 1:
                self.tag1_pos = detection.pose.pose.pose.position
            elif detection.id == 2:
                self.tag2_pos = detection.pose.pose.pose.position
        
        # Calculate distance if all tags are detected
        if self.tag0_pos and self.tag1_pos and self.tag2_pos:
            # Calculate center point of diagonal between tag1 and tag2
            center_x = (self.tag1_pos.x + self.tag2_pos.x) / 2
            center_y = (self.tag1_pos.y + self.tag2_pos.y) / 2
            center_z = (self.tag1_pos.z + self.tag2_pos.z) / 2
            
            # Calculate 2D distance (x-y plane) from tag0 to center
            distance = math.sqrt(
                (self.tag0_pos.x - center_x)**2 + 
                (self.tag0_pos.y - center_y)**2
            )
            
            # Store data
            self.timestamps.append(self.get_clock().now().seconds_nanoseconds()[0])
            self.distances.append(distance)
            
            # Keep only the latest max_data_points
            if len(self.timestamps) > self.max_data_points:
                self.timestamps = self.timestamps[-self.max_data_points:]
                self.distances = self.distances[-self.max_data_points:]
            
            # Log the distance
            self.get_logger().info(f'Distance from Tag 0 to center: {distance:.3f} meters')
    
    def update_plot(self, frame):
        if len(self.timestamps) > 1:
            # Normalize timestamps to start from 0
            t0 = self.timestamps[0]
            rel_times = [t - t0 for t in self.timestamps]
            
            # Update the plot data
            self.line.set_data(rel_times, self.distances)
            
            # Adjust the plot limits
            self.ax.set_xlim(min(rel_times), max(rel_times))
            if self.distances:
                min_dist = min(self.distances)
                max_dist = max(self.distances)
                range_dist = max_dist - min_dist
                self.ax.set_ylim(max(0, min_dist - 0.1 * range_dist), 
                                max_dist + 0.1 * range_dist)
        
        return self.line,

def main():
    rclpy.init()
    node = AprilTagDistanceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 