#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty

# read keyboad input and publish to the /destination topic
# terminated if keyboard input is 'x'

class MachineCommand(Node):
    def __init__(self):
        super().__init__('machine_command')  # Updated node name
        self.publisher_ = self.create_publisher(String, 'destination', 10)  # Updated topic name
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("Machine Command Node Started. Press 'a', 'b', 'c', or 'o' to send a message.")

    def publish_message(self, msg):
        message = String()
        message.data = msg
        self.publisher_.publish(message)
        print("\r",end="") #to start the log at the beginning of a line
        self.get_logger().info(f"Publishing: '{message.data}'")

    def read_keyboard_input(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            return key
        else:
            return None

    def run(self):
        try:
            while True:
                key = self.read_keyboard_input()
                if key in ['a', 'b', 'c', 'o']:
                    self.publish_message(key)
                # terminate, quite, exit
                if key in ['x', 'q', 'e', 't']:
                    break
        except Exception as e:
            self.get_logger().error('Error in MachineCommand: %r' % (e,))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    machine_command = MachineCommand()
    
    try:
        machine_command.run()
    except KeyboardInterrupt:
        pass
    finally:
        machine_command.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()