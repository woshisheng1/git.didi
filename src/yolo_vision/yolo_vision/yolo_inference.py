#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from hobot_dnn import pyeasy_dnn as dnn
import time
from rclpy.executors import ExternalShutdownException
from pyzbar import pyzbar
from std_msgs.msg import String
class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
        self.bridge = CvBridge()
 
        # 1. 加载地平线 BPU 模型 (.bin 文件)
        self.model_path = '/root/car_ws/src/yolo_vision/model/best.bin'  # ⚠️ 请修改为你的 bin 模型实际路径
        self.get_logger().info(f'正在加载 BPU 模型: {self.model_path}')
        self.models = dnn.load(self.model_path)
        self.model = self.models[0]
        self.get_logger().info('BPU 模型加载成功！')

        self.class_names = {
            0: "QR_code",
            1: "line",
            2: "end",
            3: "roadblock"
        }
        self.target_class_ids = {0}
        # 2. 订阅原始相机图像话题
        # ⚠️ 请修改为你实际的相机话题，例如 '/camera/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/image_raw', 
            self.image_callback,
            qos_profile_sensor_data
        )

        self.QR_publisher = self.create_publisher(
            String,
            'QR_code',
            10
        )
        # 3. 创建发布者，将画好框的图像发出去
        self.img_pub = self.create_publisher(Image, '/yolo_result_image', 10)
        
        self.last_time = time.time()
 
    # 将 BGR 图像转换为 BPU 硬件需要的 NV12 格式
    def bgr2nv12(self, bgr_image):
        height, width = bgr_image.shape[:2]
        yuv420p = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YUV_I420)
        y = yuv420p[:height, :]
        u = yuv420p[height: height + height // 4, :].reshape(height // 2, width // 2)
        v = yuv420p[height + height // 4:, :].reshape(height // 2, width // 2)
        nv12 = np.empty((height * 3 // 2, width), dtype=np.uint8)
        nv12[:height, :] = y
        nv12[height:, 0::2] = u
        nv12[height:, 1::2] = v
        return nv12
 
    # 🚀 核心优化：NumPy 矩阵加速解码，极大地降低 CPU 占用
    def decode_yolo_outputs(self, outputs, conf_thres=0.4, iou_thres=0.45):
        pred = np.squeeze(outputs[0].buffer)
        if pred.shape[0] < pred.shape[1]:
            pred = pred.transpose()
 
        # 批量过滤低置信度的框
        obj_conf = pred[:, 4]
        valid_indices = obj_conf > conf_thres
        valid_pred = pred[valid_indices]
 
        if len(valid_pred) == 0: return []
 
        cls_probs = valid_pred[:, 5:]
        class_ids = np.argmax(cls_probs, axis=1)
        scores = valid_pred[:, 4] * cls_probs[np.arange(len(valid_pred)), class_ids]
 
        score_indices = scores > conf_thres
        valid_pred = valid_pred[score_indices]
        scores = scores[score_indices]
        class_ids = class_ids[score_indices]

        if len(valid_pred) == 0: return []

        target_mask = np.isin(class_ids, list(self.target_class_ids))
        valid_pred = valid_pred[target_mask]
        scores = scores[target_mask]
        class_ids = class_ids[target_mask]

        if len(valid_pred) == 0:
            return []

        cx, cy, w, h = valid_pred[:, 0], valid_pred[:, 1], valid_pred[:, 2], valid_pred[:, 3]
        x_min, y_min = cx - w / 2, cy - h / 2
 
        boxes = np.column_stack((x_min, y_min, w, h)).astype(int).tolist()
        scores = scores.astype(float).tolist()
        class_ids = class_ids.astype(int).tolist()
 
        # NMS 非极大值抑制
        indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)
 
        results = []
        if len(indices) > 0:
            for i in indices.flatten():
                box = boxes[i]
                results.append([box[0], box[1], box[0] + box[2], box[1] + box[3], scores[i], class_ids[i]])
        return results
 
    def image_callback(self, msg):
        try:
            # 计算 FPS
            current_time = time.time()
            fps = 1.0 / (current_time - self.last_time) if (current_time - self.last_time) > 0 else 0
            self.last_time = current_time
 
            # 1. 接收 ROS 图像并转为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. 图像预处理 (缩放至模型所需大小并转为 NV12)
            resized_image = cv2.resize(cv_image, (640, 640))
            nv12_image = np.ascontiguousarray(self.bgr2nv12(resized_image))
 
            # 3. 提交 BPU 推理
            outputs = self.model.forward(nv12_image)
            
            # 4. 解析输出
            bboxes = self.decode_yolo_outputs(outputs, conf_thres=0.4)
 
            # 5. 在图像上绘制识别框
            # 5. 在图像上绘制识别框
            for box in bboxes:
                x_min, y_min, x_max, y_max, score, class_id = box

                # 👇 修改：从字典获取标签名，如果 class_id 超出范围则显示 Unknown
                label_name = self.class_names.get(class_id, f"Unknown_{class_id}")

                # 画框
                cv2.rectangle(resized_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
                if class_id == 0:
                    crop = resized_image[int(y_min):int(y_max), int(x_min):int(x_max)]
                    decoded = pyzbar.decode(crop)
                    if decoded:
                        qr_data = decoded[0].data.decode('utf-8')
                        QR_msg = String()
                        remainder = int(qr_data) % 2
                        QR_msg.data = str(remainder)
                        self.QR_publisher.publish(QR_msg)
                        self.get_logger().info(f'QR 解码结果: {remainder}')
                        # 可选：将解码内容也绘制到图像上
                        label_name = f"QR:{str(remainder)[:20]}"
                # 👇 修改：使用新的 label_name
                label_text = f"{label_name} {score:.2f}"
                cv2.putText(resized_image, label_text, (int(x_min), int(y_min) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # 6. 将画好框的图像转回 ROS 消息并发布
            result_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")
            result_msg.header = msg.header
            self.img_pub.publish(result_msg)
 
        except Exception as e:
            self.get_logger().error(f'处理图像时发生异常: {e}')
 
 
def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('检测到 Ctrl+C，节点正在退出...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()
 
if __name__ == '__main__':
    main()
