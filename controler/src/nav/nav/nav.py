import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Nav(Node):
    def __init__(self):
        super().__init__("arduino")

        self.declare_parameter("cam_id", "")

