#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np

class BoxVisualizer(Node):
    def __init__(self):
        super().__init__('box_visualizer')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tag1_pos = None
        self.tag2_pos = None
        
        # Subscribe to AprilTag detections
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/apriltag/detections',
            self.tag_callback,
            10)
        
        # Create timer to publish box frame
        self.timer = self.create_timer(0.1, self.publish_box_frame)
        
    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id == 1:  # Tag 1
                self.tag1_pos = detection.pose.pose.pose
            elif detection.id == 2:  # Tag 2
                self.tag2_pos = detection.pose.pose.pose
                
    def publish_box_frame(self):
        if self.tag1_pos is not None and self.tag2_pos is not None:
            # Create transform from tag1 to box center
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'tag_1'  # Parent frame is tag 1
            t.child_frame_id = 'box_frame'
            
            # Calculate box center (midpoint between tag1 and tag2)
            t.transform.translation.x = (self.tag1_pos.position.x + self.tag2_pos.position.x) / 2
            t.transform.translation.y = (self.tag1_pos.position.y + self.tag2_pos.position.y) / 2
            t.transform.translation.z = (self.tag1_pos.position.z + self.tag2_pos.position.z) / 2
            
            # Calculate box orientation (align with tag1)
            t.transform.rotation = self.tag1_pos.orientation
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(t)
            
            # Publish box corners
            self.publish_box_corners()
            
    def publish_box_corners(self):
        # Calculate box corners based on tag1 and tag2 positions
        tag1 = np.array([
            self.tag1_pos.position.x,
            self.tag1_pos.position.y,
            self.tag1_pos.position.z
        ])
        tag2 = np.array([
            self.tag2_pos.position.x,
            self.tag2_pos.position.y,
            self.tag2_pos.position.z
        ])
        
        # Calculate box dimensions
        box_width = np.linalg.norm(tag2 - tag1)
        
        # Calculate other corners
        corner3 = tag1 + np.array([box_width, 0, 0])
        corner4 = tag1 + np.array([0, box_width, 0])
        
        # Publish transforms for each corner
        corners = [tag1, tag2, corner3, corner4]
        for i, corner in enumerate(corners):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'tag_1'
            t.child_frame_id = f'box_corner_{i}'
            
            t.transform.translation.x = corner[0]
            t.transform.translation.y = corner[1]
            t.transform.translation.z = corner[2]
            
            t.transform.rotation = self.tag1_pos.orientation
            self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = BoxVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 