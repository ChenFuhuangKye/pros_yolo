import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import os

class pros_yolo(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
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

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # YOLO object detection
        blob = cv2.dnn.blobFromImage(image_np, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * image_np.shape[1])
                    center_y = int(detection[1] * image_np.shape[0])
                    w = int(detection[2] * image_np.shape[1])
                    h = int(detection[3] * image_np.shape[0])

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        detection_results = []

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                color = (0, 255, 0)
                cv2.rectangle(image_np, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(image_np, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                detection_info = f"label:{label}, x:{x}, y:{y}, w:{w}, h:{h}, confidence:{confidences[i]:.2f}"
                detection_results.append(detection_info)
        
        detection_str = '\n'.join(detection_results)

        detections_msg = String()
        detections_msg.data = detection_str
        self.bbox_publisher.publish(detections_msg)
        
        # Publish the image
        msg_out = CompressedImage()
        msg_out.header = msg.header
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', image_np)[1]).tobytes()
        self.publisher.publish(msg_out)        

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = pros_yolo()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
