import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import tty
import sys
import termios

class KeyboardJoyNode(Node):
    def __init__(self):
        super().__init__('keyboard_joy_node')
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.get_logger().info("Node has started.")
        self.read_keyboard()

    def read_keyboard(self):
        try:
            while True:
                key = self.get_key()
                msg = Joy()
                msg.header.stamp = self.get_clock().now().to_msg()

                # Initialize default values
                msg.axes = [0.0] * 2  # [x, y]
                msg.buttons = [0] * 12  # Assuming 12 buttons, adjust if necessary

                if key == 'w':  # Forward
                    msg.axes[1] = 1.0
                elif key == 's':  # Backward
                    msg.axes[1] = -1.0
                elif key == 'a':  # Left
                    msg.axes[0] = -1.0
                elif key == 'd':  # Right
                    msg.axes[0] = 1.0
                elif key == '\x03':
                    break

                self.joy_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error while reading keyboard input: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return ch


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJoyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
