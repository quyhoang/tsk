"""
Author: Quy Hoang, Yuta Kikuchi
Date Started: 2023 Nov 27

Description:
This node continuously reads data from an OPC UA server. When new data is received, it publishes this data to a specified topic. This topic contains the robot's destination. A separate node subscribes to this topic to control the robot's movement based on the received destination data.
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
from asyncua import Client

# Define a ROS2 Node class for continuous OPC UA communication
class OpcuaContinuousClient(Node):
    def __init__(self):
        # Initialize the node with the name 'opcua_continuous'
        super().__init__('opcua_continuous')  

        # Initialize data variables for tracking changes
        self.current_data = ''
        self.previous_data = ''

        # Set up a ROS publisher on the 'destination' topic
        self.publisher_ = self.create_publisher(String, 'destination', 10)  # Updated topic name
        
        # Log information indicating the node has started
        self.get_logger().info("OPC UA Client Node on AMR Started. Waiting for data '1', '2', '3', or '0' from PLC." )

    # Method to publish messages based on OPC data
    def publish_message(self, opcdata):
        message = String()

        # Convert OPC data to specific messages
        if opcdata == 1:
            message.data = 'a'
        elif opcdata == 2:
            message.data = 'b'
        elif opcdata == 3:
            message.data = 'c'
        elif opcdata == 0:  
            message.data = 'o'
        else:
            # Ignore data that doesn't match predefined conditions
            pass  # or any default action

        # Publish the message and log the event
        self.publisher_.publish(message)
        print("\r",end="") #to start the log at the beginning of a line
        self.get_logger().info(f"Publishing: '{message.data}'")

    # Asynchronous method to continuously fetch data from OPC UA server
    async def continuous_opc(self):
        url = 'opc.tcp://192.168.11.1:4840' #Connect OPC UA server
                                            #opc.tcp://PLC IP ADDRESS:4840
        async with Client(url = url) as client:
            while True:
                # Get the specific node from the OPC UA server
                var = client.get_node("ns=4;s=uVariable")   #ns=4;s=Variable Name(in PLC)
                                                        #ns=4 FromUaExpert
                self.previous_data = self.current_data
                value = await var.get_value()   #Get value of node as a DataValue object
                self.current_data = value

                # Publish the message only if there is new data
                if self.current_data != self.previous_data:
                    self.publish_message(value)

                # Optional: Add an asyncio sleep here for periodic fetching
                # await asyncio.sleep(1) 

def main(args=None):
    rclpy.init(args=args)
    # Create an instance of the OpcuaContinuousClient
    opcua_continuous = OpcuaContinuousClient()

    try:
        # Run the continuous OPC UA data fetching coroutine
        asyncio.run(opcua_continuous.continuous_opc())
    except KeyboardInterrupt:
        # Handle script interruption (e.g., Ctrl+C)
        pass
    finally:
        # Ensure proper cleanup of the node and ROS2 shutdown
        opcua_continuous.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()