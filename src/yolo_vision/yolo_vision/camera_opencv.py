import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class OpenCVCameraNode(Node):
    def __init__(self):
        super().__init__('opencv_camera_node')
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.get_parameter('video_device').value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('image_width').value)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('image_height').value)
        
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.033, self.timer_callback) # 限制在 30 FPS
        
        if not self.cap.isOpened():
            self.get_logger().error("无法打开相机设备！")
            exit()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()