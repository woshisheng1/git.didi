import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from rclpy.executors import ExternalShutdownException
from pyzbar.pyzbar import decode as qr_decode
from std_msgs.msg import String

# BPU 推理引擎
from hobot_dnn import pyeasy_dnn as dnn


class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')

        # 1. 参数初始化
        self.rtsp_url = self.declare_parameter('rtsp_url',
                                               'rtsp://admin:123456@192.168.9.63:8554/live').get_parameter_value().string_value
        self.model_path = self.declare_parameter('model_path',
                                                 '/root/car_ws/src/yolo_vision/model/best.bin').get_parameter_value().string_value
        self.conf_thres = self.declare_parameter('conf_thres', 0.4).get_parameter_value().double_value
        self.iou_thres = self.declare_parameter('iou_thres', 0.45).get_parameter_value().double_value

        self.bridge = CvBridge()
        self.class_names = {0: "QR_code", 1: "line", 2: "end", 3: "roadblock"}
        self.target_class_ids = {0}  # 仅针对 QR_code 进行解码

        # 2. 创建发布者，将画好框的图像发出去
        self.img_pub = self.create_publisher(Image, '/yolo_result_image', 10)
        self.QR_publisher = self.create_publisher(String, 'QR_code', 10)

        # 3. 初始化流和模型状态变量
        self.current_frame = None
        self.bboxes = []
        self.last_time = time.time()
        self.cap = None
        self.models = None

        try:
            self.get_logger().info(f'正在加载 BPU 模型: {self.model_path}')
            self.models = dnn.load(self.model_path)
            self.model = self.models[0]
            self.get_logger().info('BPU 模型加载成功！')
        except Exception as e:
            self.get_logger().error(f'模型加载失败，节点将退出: {e}')
            raise e

        try:
            self.get_logger().info(f'正在连接 RTSP 流: {self.rtsp_url}')
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if not self.cap.isOpened():
                raise Exception("无法打开 RTSP 摄像头流，请检查IP地址、端口或网络！")

            # 优化 OpenCV 拉流参数，降低延迟和 CPU 占用
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.get_logger().info('RTSP 摄像头流连接成功！')
        except Exception as e:
            self.get_logger().error(f'RTSP 连接失败，节点将退出: {e}')
            if self.cap: self.cap.release()
            raise e

        # 4. 拉流与推理分离：使用定时器触发推理，保证拉流线程不被阻塞
        self.timer = self.create_timer(0.05, self.inference_callback)

    # BGR 转 NV12 格式 (适配地平线 BPU)
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

    # 使用 NumPy 矩阵加速进行 YOLO 后处理解码
    def decode_yolo_outputs(self, outputs):
        pred = np.squeeze(outputs[0].buffer)
        if pred.shape[0] < pred.shape[1]:
            pred = pred.transpose()

        obj_conf = pred[:, 4]
        valid_indices = obj_conf > self.conf_thres
        valid_pred = pred[valid_indices]
        if len(valid_pred) == 0: return []

        cls_probs = valid_pred[:, 5:]
        class_ids = np.argmax(cls_probs, axis=1)
        scores = valid_pred[:, 4] * cls_probs[np.arange(len(valid_pred)), class_ids]

        score_indices = scores > self.conf_thres
        valid_pred = valid_pred[score_indices]
        scores = scores[score_indices]
        class_ids = class_ids[score_indices]

        if len(valid_pred) == 0: return []

        target_mask = np.isin(class_ids, list(self.target_class_ids))
        valid_pred = valid_pred[target_mask]
        scores = scores[target_mask]
        class_ids = class_ids[target_mask]
        if len(valid_pred) == 0: return []

        cx, cy, w, h = valid_pred[:, 0], valid_pred[:, 1], valid_pred[:, 2], valid_pred[:, 3]
        x_min, y_min = cx - w / 2, cy - h / 2

        boxes = np.column_stack((x_min, y_min, w, h)).astype(int).tolist()
        scores = scores.astype(float).tolist()
        class_ids = class_ids.astype(int).tolist()

        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, self.iou_thres)

        results = []
        if len(indices) > 0:
            for i in indices.flatten():
                box = boxes[i]
                results.append([box[0], box[1], box[0] + box[2], box[1] + box[3], scores[i], class_ids[i]])
        return results

    def inference_callback(self):
        """定时触发 BPU 推理和 ROS 发布"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("读取视频帧失败，尝试重连...")
            return

        try:
            # 计算 FPS
            current_time = time.time()
            fps = 1.0 / (current_time - self.last_time) if (current_time - self.last_time) > 0 else 0
            self.last_time = current_time

            # 1. 图像预处理
            resized_image = cv2.resize(frame, (640, 640))
            nv12_image = np.ascontiguousarray(self.bgr2nv12(resized_image))

            # 2. 提交 BPU 推理
            outputs = self.model.forward(nv12_image)

            # 3. 解析输出
            bboxes = self.decode_yolo_outputs(outputs)

            # 4. 绘制识别框并执行二维码解码
            for box in bboxes:
                x_min, y_min, x_max, y_max, score, class_id = box
                label_name = self.class_names.get(class_id, f"Unknown_{class_id}")

                # 画框
                cv2.rectangle(resized_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)

                # 仅对 QR_code (class_id == 0) 进行 ROI 裁剪和解码
                if class_id == 0:
                    crop = resized_image[int(y_min):int(y_max), int(x_min):int(x_max)]
                    decoded = qr_decode(crop)
                    if decoded:
                        qr_data = decoded[0].data.decode('utf-8')
                        remainder = int(qr_data) % 2
                        QR_msg = String()
                        QR_msg.data = str(remainder)
                        self.QR_publisher.publish(QR_msg)
                        self.get_logger().info(f'QR 解码成功并发布: {remainder} (原始数据: {qr_data})')
                        label_name = f"QR:{str(remainder)}"

                # 绘制标签文本
                label_text = f"{label_name} {score:.2f}"
                cv2.putText(resized_image, label_text, (int(x_min), int(y_min) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 打印 FPS
                cv2.putText(resized_image, f"FPS: {fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # 5. 发布处理后的图像消息
                result_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")
                self.img_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'处理图像时发生异常: {e}')

    def destroy_node(self):
        """优雅退出时释放资源"""
        if self.cap:
            self.cap.release()
            self.get_logger().info('RTSP 摄像头流已释放')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('检测到退出信号，节点正在安全关闭...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()