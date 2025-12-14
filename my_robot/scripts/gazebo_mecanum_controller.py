#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class GazeboMecanumController(Node):
    def __init__(self):
        super().__init__('gazebo_mecanum_controller')
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publishers for each wheel joint velocity
        self.wheel_pubs = {
            'front_left': self.create_publisher(Float64, '/front_left_wheel_joint_velocity_controller/command', 10),
            'front_right': self.create_publisher(Float64, '/front_right_wheel_joint_velocity_controller/command', 10),
            'rear_left': self.create_publisher(Float64, '/rear_left_wheel_joint_velocity_controller/command', 10),
            'rear_right': self.create_publisher(Float64, '/rear_right_wheel_joint_velocity_controller/command', 10),
        }
        
        # Robot parameters
        self.wheel_radius = 0.1  # meters
        self.wheel_separation_width = 0.5  # distance between left and right wheels
        self.wheel_separation_length = 0.5  # distance between front and rear wheels
        
        self.get_logger().info('Gazebo Mecanum Controller Started')

    def cmd_vel_callback(self, msg):
        """Convert Twist to individual wheel velocities for mecanum drive"""
        # Extract commanded velocities
        vx = msg.linear.x  # forward/backward
        vy = msg.linear.y  # left/right strafe
        wz = msg.angular.z  # rotation
        
        # Mecanum wheel kinematics
        # For each wheel: v = vx ± vy ± wz * (lx + ly)/2
        # lx = wheel_separation_length/2
        # ly = wheel_separation_width/2
        
        lx_ly = (self.wheel_separation_length + self.wheel_separation_width) / 2.0
        
        # Calculate wheel velocities
        v_fl = (vx - vy - wz * lx_ly) / self.wheel_radius  # Front Left
        v_fr = (vx + vy + wz * lx_ly) / self.wheel_radius  # Front Right
        v_rl = (vx + vy - wz * lx_ly) / self.wheel_radius  # Rear Left
        v_rr = (vx - vy + wz * lx_ly) / self.wheel_radius  # Rear Right
        
        # Publish wheel velocities
        self.publish_wheel_velocity('front_left', v_fl)
        self.publish_wheel_velocity('front_right', v_fr)
        self.publish_wheel_velocity('rear_left', v_rl)
        self.publish_wheel_velocity('rear_right', v_rr)
        
        self.get_logger().debug(f'Commanded: vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}')
        self.get_logger().debug(f'Wheels: FL={v_fl:.2f} FR={v_fr:.2f} RL={v_rl:.2f} RR={v_rr:.2f}')

    def publish_wheel_velocity(self, wheel_name, velocity):
        """Publish velocity command to a wheel"""
        msg = Float64()
        msg.data = float(velocity)
        self.wheel_pubs[wheel_name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboMecanumController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
