#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

class YOLOFaceDetector(Node):
    def __init__(self):
        super().__init__('yolo_face_detector')
        
        if not YOLO_AVAILABLE:
            self.get_logger().error('Ultralytics not installed! Run: bash ~/nero/install_yolo.sh')
            return
            
        self.get_logger().info('Loading YOLOv8n model (lightweight)...')
        self.model = YOLO('yolov8n.pt')  # Nano - fastest
        self.get_logger().info('YOLO ready - detecting people only')
        
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # Subscribe to camera
        self.sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        self.pub = self.create_publisher(Image, '/camera/detections', 10)
        
        self.get_logger().info('Face detector started - publishing to /camera/detections')
    
    def image_callback(self, msg):
        if not YOLO_AVAILABLE:
            return
        
        # Process every 3rd frame to reduce load (10 FPS instead of 30)
        self.frame_count += 1
        if self.frame_count % 3 != 0:
            return
            
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO - only detect person class (class 0)
            results = self.model(cv_image, verbose=False, classes=[0], conf=0.5)
            
            # Draw only person detections
            annotated_frame = results[0].plot()
            
            # Add text overlay
            cv2.putText(annotated_frame, f'People detected: {len(results[0].boxes)}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Convert back to ROS Image
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            detection_msg.header = msg.header
            
            # Publish
            self.pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    if not YOLO_AVAILABLE:
        print('ERROR: ultralytics not installed!')
        print('Install with: bash ~/nero/install_yolo.sh')
        rclpy.shutdown()
        return
    
    detector = YOLOFaceDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
