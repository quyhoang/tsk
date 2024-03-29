#!/usr/bin/env python3

"""
Author: Quy Hoang
Date Started: Nov 22, 2023

Description:
This script defines the 'MachineCommand' node, which reads keyboard inputs and publishes them to the '/destination' topic. It is primarily used for sending simple control commands ('a', 'b', 'c', 'o') to a robot. The node remains active until it receives an exit command ('x', 'q', 'e', 't'). This setup allows for easy and direct control of robotic systems via keyboard inputs.
"""

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
        self.get_logger().info("\nMachine Command Node Started. \nPress 'a', 'b', 'c', or 'o' to send a message. \nCtrl C to terminate.")

    def publish_message(self, msg):
        message = String()
        message.data = msg
        self.publisher_.publish(message)
        # print("\r",end="") #to start the log at the beginning of a line
        self.get_logger().info(f"Publishing: '{message.data}'")
        print("\r",end="") #to start the log at the beginning of a line

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
                if (key == '\x03'): #Control + C
                # if key in ['x', 'q', 'e', 't']:
                    print("\r",end="")
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