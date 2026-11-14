import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class QR_Subscriber(Node):
    def __init__(self):
        super().__init__('qr_subscriber')
        self.subscription = self.create_subscription(
            String,
            'QR_code',
            self.listener_callback,
            10
        )

    def listener_callback(self,QR_msg):
        self.get_logger().info(f'得到解码结果为{QR_msg.data}')

def main(args = None):
    rclpy.init(args = args)
    node = QR_Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()