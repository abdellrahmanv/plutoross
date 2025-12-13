#!/usr/bin/env python3
"""
Mecanum Wheel Controller for RViz2 Visualization
Subscribes to /cmd_vel and publishes joint states for 4 mecanum wheels
No-slip assumption for visual feedback
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        
        # Robot parameters (adjust based on your robot dimensions)
        self.wheel_radius = 0.1  # meters
        self.robot_width = 0.5   # distance between left and right wheels
        self.robot_length = 0.5  # distance between front and back wheels
        
        # Wheel joint names (from your URDF)
        self.wheel_joint_names = [
            'front_leftwheelj',
            'front_rightwheelj',
            'back_leftwheelj', 
            'back_rigtwheelj'
        ]
        
        # Arm joint names
        self.arm_joint_names = [
            'leftarmj',
            'rightarmj'
        ]
        
        # All joint names
        self.joint_names = self.wheel_joint_names + self.arm_joint_names
        
        # Current positions (radians)
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.arm_positions = [0.0, 0.0]  # Arms stay at zero
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer for continuous publishing (50Hz)
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        self.get_logger().info('Mecanum Controller Started!')
        self.get_logger().info('Listening for /cmd_vel commands...')
        self.get_logger().info(f'Wheel joints: {self.joint_names}')
        
    def cmd_vel_callback(self, msg):
        """
        Convert Twist command to mecanum wheel velocities
        
        Mecanum wheel equations (no-slip):
        vx = linear.x (forward/backward)
        vy = linear.y (strafe left/right)
        wz = angular.z (rotation)
        
        Wheel velocities:
        front_left  = vx - vy - wz * (width + length) / 2
        front_right = vx + vy + wz * (width + length) / 2
        back_left   = vx + vy - wz * (width + length) / 2
        back_right  = vx - vy + wz * (width + length) / 2
        """
        
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Calculate diagonal distance for rotation
        l = (self.robot_width + self.robot_length) / 2.0
        
        # Mecanum wheel velocities (m/s)
        v_fl = vx - vy - wz * l
        v_fr = vx + vy + wz * l
        v_bl = vx + vy - wz * l
        v_br = vx - vy + wz * l
        
        # Convert linear velocity to angular velocity (rad/s)
        # wheel_angular_vel = linear_vel / wheel_radius
        wheel_vels = [
            v_fl / self.wheel_radius,
            v_fr / self.wheel_radius,
            v_bl / self.wheel_radius,
            v_br / self.wheel_radius
        ]
        
        # Update wheel positions (integrate velocity over time)
        dt = 0.02  # 50Hz = 0.02s
        for i in range(4): + self.arm_positions
        msg.velocity = [0.0] * 6  # 4 wheels + 2 arms
        msg.effort = [0.0] * 6
        self.get_logger().debug(f'Velocities: FL={v_fl:.2f} FR={v_fr:.2f} BL={v_bl:.2f} BR={v_br:.2f}')
        
    def publish_joint_states(self):
        """Publish current joint states for visualization"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.wheel_positions
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = MecanumController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
