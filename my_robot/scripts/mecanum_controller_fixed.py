#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        
        # Wheel parameters
        self.wheel_radius = 0.05  # 5cm
        self.wheel_base_width = 0.3  # 30cm  
        self.wheel_base_length = 0.4  # 40cm
        
        # Wheel joint names - ONLY the 4 wheels
        self.joint_names = [
            'front_leftwheelj',
            'front_rightwheelj',
            'back_leftwheelj',
            'back_rigtwheelj'
        ]
        
        # Track positions
        self.positions = [0.0, 0.0, 0.0, 0.0]
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        self.get_logger().info('Mecanum Controller Started!')
        self.get_logger().info(f'Publishing joints: {self.joint_names}')
    
    def cmd_vel_callback(self, msg):
        # Extract velocities
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Mecanum wheel kinematics
        width = self.wheel_base_width
        length = self.wheel_base_length
        
        # Calculate wheel velocities (rad/s)
        front_left  = (vx - vy - wz * (width + length) / 2) / self.wheel_radius
        front_right = (vx + vy + wz * (width + length) / 2) / self.wheel_radius
        back_left   = (vx + vy - wz * (width + length) / 2) / self.wheel_radius
        back_right  = (vx - vy + wz * (width + length) / 2) / self.wheel_radius
        
        # Integrate velocities to positions (simple)
        dt = 0.02
        self.positions[0] += front_left * dt
        self.positions[1] += front_right * dt
        self.positions[2] += back_left * dt
        self.positions[3] += back_right * dt
    
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = self.joint_names
        msg.position = self.positions
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
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
