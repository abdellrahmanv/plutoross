#!/usr/bin/env python3
"""
Simple arm controller for 1DOF grasping arms
Publishes effort commands to Gazebo joint effort controller plugins
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64

class GazeboArmController(Node):
    def __init__(self):
        super().__init__('gazebo_arm_controller')
        
        # Subscribe to arm velocity commands from keyboard
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'cmd_arm_vel',
            self.arm_vel_callback,
            10)
        
        # Publishers for joint effort commands
        self.left_pub = self.create_publisher(Float64, '/leftarmj_effort', 10)
        self.right_pub = self.create_publisher(Float64, '/rightarmj_effort', 10)
        
        # Effort scaling factor (velocity to effort)
        self.effort_scale = 5.0
        
        self.get_logger().info('✓ Gazebo Arm Controller Started')
        self.get_logger().info('✓ Publishing efforts to /leftarmj_effort and /rightarmj_effort')
        self.get_logger().info('✓ Ready for U/J/I/K commands!')

    def arm_vel_callback(self, msg):
        """Convert velocity commands to effort and publish"""
        if len(msg.data) >= 2:
            left_vel = msg.data[0]
            right_vel = msg.data[1]
            
            # Convert velocity to effort
            left_effort = left_vel * self.effort_scale
            right_effort = right_vel * self.effort_scale
            
            # Publish efforts
            left_msg = Float64()
            left_msg.data = float(left_effort)
            self.left_pub.publish(left_msg)
            
            right_msg = Float64()
            right_msg.data = float(right_effort)
            self.right_pub.publish(right_msg)
            
            # Log significant commands
            if abs(left_vel) > 0.01 or abs(right_vel) > 0.01:
                self.get_logger().info(
                    f'Arms: L={left_effort:.1f}Nm R={right_effort:.1f}Nm')

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
