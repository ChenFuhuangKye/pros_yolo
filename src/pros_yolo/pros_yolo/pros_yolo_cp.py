import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import os

class ProsYOLO(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/image/yolo',
            10)
        self.bbox_publisher = self.create_publisher(
            String,
            '/camera/bbox',
            10)

        dir = os.path.dirname(__file__)
        weights = os.path.join(dir, 'yolov3.weights')
        cfg = os.path.join(dir, 'yolov3.cfg')
        names = os.path.join(dir, 'coco.names')

        self.net = cv2.dnn.readNet(weights, cfg)

        with open(names, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4

        self.batch_size = 4  # 批次處理大小
        self.image_batch = []
        self.header_batch = []

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image_np is None:
            self.get_logger().error('Failed to decode image')
            return

        self.image_batch.append(image_np)
        self.header_batch.append(msg.header)

        if len(self.image_batch) == self.batch_size:
            self.process_batch()

    def process_batch(self):
        # 建立批次圖像的blob
        blobs = [cv2.dnn.blobFromImage(img, 0.00392, (320, 320), (0, 0, 0), True, crop=False) for img in self.image_batch]
        blob = np.vstack(blobs)

        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # outs的形狀應該是 (batch_size, num_boxes, attributes)
        if len(outs) != self.batch_size:
            self.get_logger().error('Unexpected output shape from the network')
            self.image_batch = []
            self.header_batch = []
            return

        detection_results_batch = []
        processed_images = []

        for i in range(self.batch_size):
            class_ids = []
            confidences = []
            boxes = []

            for detection in outs[i]:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.confidence_threshold:
                    center_x = int(detection[0] * self.image_batch[i].shape[1])
                    center_y = int(detection[1] * self.image_batch[i].shape[0])
                    w = int(detection[2] * self.image_batch[i].shape[1])
                    h = int(detection[3] * self.image_batch[i].shape[0])

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

            indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)
            detection_results = []

            for j in range(len(boxes)):
                if j in indexes:
                    x, y, w, h = boxes[j]
                    label = str(self.classes[class_ids[j]])
                    color = (0, 255, 0)
                    cv2.rectangle(self.image_batch[i], (x, y), (x + w, y + h), color, 2)
                    cv2.putText(self.image_batch[i], label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    detection_info = f"label:{label}, x:{x}, y:{y}, w:{w}, h:{h}, confidence:{confidences[j]:.2f}"
                    detection_results.append(detection_info)

            detection_results_batch.append('\n'.join(detection_results))
            processed_images.append(self.image_batch[i])

        for i in range(self.batch_size):
            detections_msg = String()
            detections_msg.data = detection_results_batch[i]
            self.bbox_publisher.publish(detections_msg)

            msg_out = CompressedImage()
            msg_out.header = self.header_batch[i]
            msg_out.format = "jpeg"
            msg_out.data = np.array(cv2.imencode('.jpg', processed_images[i])[1]).tobytes()
            self.publisher.publish(msg_out)

        self.image_batch = []
        self.header_batch = []

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ProsYOLO()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
