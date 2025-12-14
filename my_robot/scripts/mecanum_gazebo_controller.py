#!/usr/bin/env python3
"""
Mecanum Controller for Gazebo Simulation
This controller reads /cmd_vel from keyboard teleop and publishes arm commands
The base movement is handled by libgazebo_ros_planar_move.so plugin in Gazebo
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class MecanumGazeboController(Node):
    def __init__(self):
        super().__init__('mecanum_gazebo_controller')
        
        # Current arm positions
        self.arm_positions = [0.0, 0.0]
        self.arm_velocities = [0.0, 0.0]
        
        # Subscriber for arm velocity commands
        self.cmd_arm_sub = self.create_subscription(
            Float64MultiArray, 'cmd_arm_vel', self.cmd_arm_callback, 10)
        
        # Publisher for joint states (arms only - wheels handled by Gazebo)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to update at 50Hz
        self.timer = self.create_timer(0.02, self.update)
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Mecanum Gazebo Controller Started!')
        self.get_logger().info('Base movement handled by Gazebo planar_move plugin')
        self.get_logger().info('Arm control via /cmd_arm_vel topic')
    
    def cmd_arm_callback(self, msg):
        if len(msg.data) >= 2:
            self.arm_velocities = list(msg.data[:2])
    
    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Update arm positions
        self.arm_positions[0] += self.arm_velocities[0] * dt
        self.arm_positions[1] += self.arm_velocities[1] * dt
        
        # Clamp arm positions to limits
        self.arm_positions[0] = max(-3.14, min(3.14, self.arm_positions[0]))
        self.arm_positions[1] = max(-3.14, min(3.14, self.arm_positions[1]))
        
        # Publish arm joint states
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = ['leftarmj', 'rightarmj']
        msg.position = self.arm_positions
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = MecanumGazeboController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
