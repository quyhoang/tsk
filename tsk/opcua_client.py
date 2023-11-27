"""
Author: Quy Hoang
Date Started: 2023 Nov 27

Description:
This node periodically reads data from an OPC UA server. When new data is received, it publishes this data to a specified topic. This topic contains the robot's destination. A separate node subscribes to this topic to control the robot's movement based on the received destination data.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from opcua import Client


class OpcuaClient(Node):
    def __init__(self):

        super().__init__('opcua_client')  # Updated node name
        self.current_data = ''
        self.previous_data = ''
        self.publisher_ = self.create_publisher(String, 'destination', 10)  # Updated topic name
        self.timer = self.create_timer(1.0, self.timer_callback)  # 5.0 seconds interval
        self.client = Client("opc.tcp://192.168.0.10:4840")
        # self.client.session_timeout = 99999999999
        # self.client.connect()
        
        self.get_logger().info("OPC UA Client Node on AMR Started. Waiting for data '1', '2', '3', or '0' from PLC." )

    def publish_message(self, opcdata):
        message = String()
        if opcdata == 1:
            message.data = 'a'
        elif opcdata == 2:
            message.data = 'b'
        elif opcdata == 3:
            message.data = 'c'
        elif opcdata == 0:  
            message.data = 'o'
        else:
            # Handle the case where none of the above conditions are met
            pass  # or any default action

        self.publisher_.publish(message)
        print("\r",end="") #to start the log at the beginning of a line
        self.get_logger().info(f"Publishing: '{message.data}'")

    def timer_callback(self):
        self.previous_data = self.current_data
        self.client.connect()
        try:
            var = self.client.get_node("ns=4;s=uVariable")
            value = var.get_value()

            # Only publish if new data is received
            self.current_data = value
            if self.current_data != self.previous_data:
                self.publish_message(value)

        except Exception as e:
            self.get_logger().error(f"Error while fetching data: {e}")
            # Consider reconnecting or handling the error as needed
        
        finally:
            self.client.disconnect()
        self.client.disconnect()

    # def run(self):
    #     client = Client("opc.tcp://192.168.0.10:4840")  #Connect OPC UA server
    #                                                 #opc.tcp://PLC IP ADDRESS:4840
    #     client.connect()
    #     try:
    #         while True:
    #             var = client.get_node("ns=4;s=uVariable")   #ns=4;s=Variable Name(in PLC)
    #                                                         #ns=4 FromUaExpert
    #             value = var.get_value()                     #Get value of node as a DataValue object
    #             self.publish_message(value)
    #     except KeyboardInterrupt:
    #         rclpy.shutdown()
    #     finally:
    #         client.disconnect()

def main(args=None):
    rclpy.init(args=args)
    opcua_client = OpcuaClient()
    
    try:
        rclpy.spin(opcua_client)
    except KeyboardInterrupt:
        pass
    finally:
        opcua_client.destroy_node()
        # opcua_client.client.disconnect()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


