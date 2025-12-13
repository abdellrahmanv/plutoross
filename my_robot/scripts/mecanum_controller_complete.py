#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')

        # Wheel parameters
        self.wheel_radius = 0.05  # 5cm
        self.wheel_base_width = 0.3  # 30cm
        self.wheel_base_length = 0.4  # 40cm

        # ALL joint names - 4 wheels + 2 arms
        self.joint_names = [
            'front_leftwheelj',
            'front_rightwheelj',
            'back_leftwheelj',
            'back_rigtwheelj',
            'leftarmj',
            'rightarmj'
        ]

        # Robot State
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.arm_positions = [0.0, 0.0]  # Arms at rest position
        self.arm_velocities = [0.0, 0.0] # Arm velocities
        
        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Current velocities
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Subscriber for arm velocity commands
        self.cmd_arm_sub = self.create_subscription(
            Float64MultiArray, 'cmd_arm_vel', self.cmd_arm_callback, 10)

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish joint states and TF at 50Hz
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info('Mecanum Mobile Base Controller Started!')
        self.get_logger().info('Publishing: JointStates + Odometry TF')

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def cmd_arm_callback(self, msg):
        if len(msg.data) >= 2:
            self.arm_velocities = list(msg.data[:2])

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. Update Odometry (Global Position)
        # Simple Euler integration for position
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 2. Update Wheel Positions (for JointStates)
        width = self.wheel_base_width
        length = self.wheel_base_length
        
        # Mecanum inverse kinematics for wheel velocities
        front_left  = (self.vx - self.vy - self.wz * (width + length) / 2) / self.wheel_radius
        front_right = (self.vx + self.vy + self.wz * (width + length) / 2) / self.wheel_radius
        back_left   = (self.vx + self.vy - self.wz * (width + length) / 2) / self.wheel_radius
        back_right  = (self.vx - self.vy + self.wz * (width + length) / 2) / self.wheel_radius

        self.wheel_positions[0] += front_left * dt
        self.wheel_positions[1] += front_right * dt
        self.wheel_positions[2] += back_left * dt
        self.wheel_positions[3] += back_right * dt

        # 3. Update Arm Positions
        self.arm_positions[0] += self.arm_velocities[0] * dt
        self.arm_positions[1] += self.arm_velocities[1] * dt

        # Clamp arm positions to limits (-3.14 to 3.14)
        self.arm_positions[0] = max(-3.14, min(3.14, self.arm_positions[0]))
        self.arm_positions[1] = max(-3.14, min(3.14, self.arm_positions[1]))

        # 4. Publish JointState
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = self.joint_names
        msg.position = self.wheel_positions + self.arm_positions
        self.joint_pub.publish(msg)

        # 5. Broadcast TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Quaternion from Yaw
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.th / 2)
        t.transform.rotation.w = math.cos(self.th / 2)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    controller = MecanumController()
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
