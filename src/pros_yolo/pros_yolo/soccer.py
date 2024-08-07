import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
import cv2
import numpy as np

class pros_yolo(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.get_logger().info('Node is running')
        self.bridge = CvBridge()

        self.model = YOLO("/workspaces/src/pros_yolo/pros_yolo/model/best.pt")
        self.subscription = self.create_subscription(
                CompressedImage,
                '/camera/image/compressed',
                self.listener_callback,
                10)
        
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/image/yolo',
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Received frame')
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        results = self.model.track(frame, persist=True)  # convert to track mode

        # visualizing results
        annotated_frame = results[0].plot()

        # show the frame
        cv2.imshow("YOLOv8 Tracking", annotated_frame)

        # encode frame as CompressedImage and publish
        msg_out = CompressedImage()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', annotated_frame)[1]).tobytes()
        self.publisher.publish(msg_out)

        # click 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = pros_yolo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
