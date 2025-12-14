#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        if not YOLO_AVAILABLE:
            self.get_logger().error('Ultralytics not installed! Run: pip3 install ultralytics')
            return
            
        self.get_logger().info('Loading YOLOv8 model...')
        self.model = YOLO('yolov8n.pt')  # Nano model for speed
        self.get_logger().info('YOLOv8 ready!')
        
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        self.pub = self.create_publisher(Image, '/camera/detections', 10)
        
        self.get_logger().info('YOLO detector started - publishing to /camera/detections')
    
    def image_callback(self, msg):
        if not YOLO_AVAILABLE:
            return
            
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model(cv_image, verbose=False)
            
            # Draw detections on image
            annotated_frame = results[0].plot()
            
            # Convert back to ROS Image
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            detection_msg.header = msg.header
            
            # Publish annotated image
            self.pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    if not YOLO_AVAILABLE:
        print('ERROR: ultralytics not installed!')
        print('Install with: pip3 install ultralytics opencv-python')
        rclpy.shutdown()
        return
    
    detector = YOLODetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
