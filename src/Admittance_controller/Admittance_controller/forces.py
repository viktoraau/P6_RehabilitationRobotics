#!/usr/bin/env python3
"""
Interactive Force Tester for Admittance Controller
Allows keyboard control to generate test forces/torques in different directions.

Usage:
  ros2 run p4_admittance_controller interactive_force_tester

Controls:
  Arrow Keys: Force in X/Y directions
  Q/A: Force in Z direction (up/down)
  W/S: Torque around X axis
  E/D: Torque around Y axis
  R/F: Torque around Z axis
  
  +/-: Increase/decrease magnitude
  Space: Stop all forces
  Esc: Quit
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class InteractiveForceTester(Node):
    def __init__(self):
        super().__init__('interactive_force_tester')
        
        # Parameters
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('initial_magnitude', 1.0)  # N or Nm
        self.declare_parameter('magnitude_step', 0.1)  # increment/decrement step
        self.declare_parameter('topic', '/ft300/wrench')
        
        self.rate = float(self.get_parameter('publish_rate').value)
        self.magnitude = float(self.get_parameter('initial_magnitude').value)
        self.mag_step = float(self.get_parameter('magnitude_step').value)
        topic = self.get_parameter('topic').value
        
        # Publisher
        self.pub = self.create_publisher(WrenchStamped, topic, 10)
        
        # Current force/torque command [fx, fy, fz, tx, ty, tz]
        self.wrench = [0.0] * 6
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.rate, self.publish_wrench)
        
        # Print instructions
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*70)
        print("  Interactive Force/Torque Tester for Admittance Controller")
        print("="*70)
        print("\nKeyboard Controls:")
        print("  ↑/↓  : Apply force in +X/-X direction")
        print("  ←/→  : Apply force in +Y/-Y direction")
        print("  Q/A  : Apply force in +Z/-Z direction (up/down)")
        print("\n  W/S  : Apply torque around X axis")
        print("  E/D  : Apply torque around Y axis")
        print("  R/F  : Apply torque around Z axis")
        print("\n  +/=  : Increase magnitude")
        print("  -/_  : Decrease magnitude")
        print("  Space: Reset all forces to zero")
        print("  Esc/q: Quit")
        print("="*70)
        print(f"\nCurrent magnitude: {self.magnitude:.1f} N/Nm")
        print("\nPress any key to start...\n")
        
    def publish_wrench(self):
        """Publish current wrench at the specified rate."""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.wrench.force.x = self.wrench[0]
        msg.wrench.force.y = self.wrench[1]
        msg.wrench.force.z = self.wrench[2]
        msg.wrench.torque.x = self.wrench[3]
        msg.wrench.torque.y = self.wrench[4]
        msg.wrench.torque.z = self.wrench[5]
        
        self.pub.publish(msg)
        
    def update_display(self):
        """Update the terminal display with current status."""
        print(f"\r[Mag: {self.magnitude:5.1f}] Fx:{self.wrench[0]:6.1f} Fy:{self.wrench[1]:6.1f} Fz:{self.wrench[2]:6.1f} "
              f"Tx:{self.wrench[3]:6.2f} Ty:{self.wrench[4]:6.2f} Tz:{self.wrench[5]:6.2f}   ", end='', flush=True)
    
    def handle_key(self, key):
        """Handle keyboard input."""
        changed = False
        
        # Force commandsbase_link
        if key == '\x1b[A':  # Up arrow
            self.wrench[0] = self.magnitude
            changed = True
        elif key == '\x1b[B':  # Down arrow
            self.wrench[0] = -self.magnitude
            changed = True
        elif key == '\x1b[C':  # Right arrow
            self.wrench[1] = self.magnitude
            changed = True
        elif key == '\x1b[D':  # Left arrow
            self.wrench[1] = -self.magnitude
            changed = True
        elif key.lower() == 'q':
            self.wrench[2] = self.magnitude
            changed = True
        elif key.lower() == 'a':
            self.wrench[2] = -self.magnitude
            changed = True
            
        # Torque commands
        elif key.lower() == 'w':
            self.wrench[3] = self.magnitude * 0.1  # Scale down for torques
            changed = True
        elif key.lower() == 's':
            self.wrench[3] = -self.magnitude * 0.1
            changed = True
        elif key.lower() == 'e':
            self.wrench[4] = self.magnitude * 0.1
            changed = True
        elif key.lower() == 'd':
            self.wrench[4] = -self.magnitude * 0.1
            changed = True
        elif key.lower() == 'r':
            self.wrench[5] = self.magnitude * 0.1
            changed = True
        elif key.lower() == 'f':
            self.wrench[5] = -self.magnitude * 0.1
            changed = True
            
        # Magnitude adjustment
        elif key in ['+', '=']:
            self.magnitude += self.mag_step
            print(f"\nMagnitude increased to {self.magnitude:.1f} N/Nm")
            changed = True
        elif key in ['-', '_']:
            self.magnitude = max(0.0, self.magnitude - self.mag_step)
            print(f"\nMagnitude decreased to {self.magnitude:.1f} N/Nm")
            changed = True
            
        # Reset
        elif key == ' ':
            self.wrench = [0.0] * 6
            print("\nAll forces reset to zero")
            changed = True
            
        # Quit
        elif key == '\x1b' or key.lower() == 'q':
            return False
            
        if changed:
            self.update_display()
            
        return True


def get_key():
    """Get a single keypress from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        
        # Read first character
        ch = sys.stdin.read(1)
        
        # Handle escape sequences (arrow keys, etc.)
        if ch == '\x1b':
            # Read next two characters for arrow keys
            next1 = sys.stdin.read(1)
            next2 = sys.stdin.read(1)
            ch = ch + next1 + next2
            
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveForceTester()
    
    # Run node in a separate thread
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Main keyboard loop
        while rclpy.ok():
            key = get_key()
            if not node.handle_key(key):
                break
                
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\n\nShutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()