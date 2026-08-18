import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import threading



class serial_bridge():
    def __init__(self):
        super().__init__("serial_bridge")
        # Wichtig: Der Name muss exakt übereinstimmen!
        self.pub_uart_stm = self.create_publisher(String, '/uart_stm', 10) 
        self.get_logger().info(f"Empfangen auf /uart_stm: MOVE::S{cmd_msg.data}\n")

    def callback(self):
        x=cmd_msg.data