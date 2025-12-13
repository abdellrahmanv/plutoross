#!/usr/bin/env python3
"""
Simple keyboard teleop for mecanum wheels
Controls:
  w/s - forward/backward
  a/d - strafe left/right
  q/e - rotate left/right
  space - stop
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_publisher = self.create_publisher(Float64MultiArray, 'cmd_arm_vel', 10)

        self.linear_speed = 0.5  # m/s
        self.strafe_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.arm_speed = 1.0 # rad/s

        self.get_logger().info('Keyboard Teleop Started!')
        self.print_instructions()
        
    def print_instructions(self):
        msg = """
╔═══════════════════════════════════════╗
║   MECANUM WHEEL & ARM CONTROL        ║
╠═══════════════════════════════════════╣
║  W - Forward          Q - Rotate CCW  ║
║  S - Backward         E - Rotate CW   ║
║  A - Strafe Left                      ║
║  D - Strafe Right                     ║
║                                       ║
║  U/J - Left Arm Up/Down               ║
║  I/K - Right Arm Up/Down              ║
║                                       ║
║  SPACE - Stop All                     ║
║  Ctrl+C - Quit                        ║
╚═══════════════════════════════════════╝
        """
        print(msg)
        
    def get_key(self):
        """Get single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
        
    def run(self):
        """Main control loop"""
        while rclpy.ok():
            key = self.get_key()
            
            twist = Twist()
            arm_vel = Float64MultiArray()
            arm_vel.data = [0.0, 0.0]

            if key == 'w':
                twist.linear.x = self.linear_speed
                self.get_logger().info('Forward')
            elif key == 's':
                twist.linear.x = -self.linear_speed
                self.get_logger().info('Backward')
            elif key == 'a':
                twist.linear.y = self.strafe_speed
                self.get_logger().info('Strafe Left')
            elif key == 'd':
                twist.linear.y = -self.strafe_speed
                self.get_logger().info('Strafe Right')
            elif key == 'q':
                twist.angular.z = self.angular_speed
                self.get_logger().info('Rotate CCW')
            elif key == 'e':
                twist.angular.z = -self.angular_speed
                self.get_logger().info('Rotate CW')
            elif key == 'u':
                arm_vel.data = [self.arm_speed, 0.0]
                self.get_logger().info('Left Arm UP')
            elif key == 'j':
                arm_vel.data = [-self.arm_speed, 0.0]
                self.get_logger().info('Left Arm DOWN')
            elif key == 'i':
                arm_vel.data = [0.0, self.arm_speed]
                self.get_logger().info('Right Arm UP')
            elif key == 'k':
                arm_vel.data = [0.0, -self.arm_speed]
                self.get_logger().info('Right Arm DOWN')
            elif key == ' ':
                self.get_logger().info('STOP')
            elif key == '\x03':  # Ctrl+C
                break
            else:
                continue

            self.publisher.publish(twist)
            self.arm_publisher.publish(arm_vel)

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()

    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        twist = Twist()
        teleop.publisher.publish(twist)
        
        arm_vel = Float64MultiArray()
        arm_vel.data = [0.0, 0.0]
        teleop.arm_publisher.publish(arm_vel)
        

if __name__ == '__main__':
    main()
