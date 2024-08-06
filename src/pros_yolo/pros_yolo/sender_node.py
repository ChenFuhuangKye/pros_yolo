import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np
import ffmpeg
import cv2
import queue
import threading
import io

class ImageToVP9Sender(Node):
    def __init__(self):
        super().__init__('image_to_vp9_sender')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(ByteMultiArray, '/vp9_stream', 10)
        self.image_queue = queue.Queue()
        self.image_count = 0
        self.process_thread = threading.Thread(target=self.process_images)
        self.process_thread.start()
        self.get_logger().info("ImageToVP9Sender node started")

    def image_callback(self, data):
        self.get_logger().info(f"Received image {self.image_count}")
        
        try:
            # 解压缩图像数据
            np_arr = np.frombuffer(data.data, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 检查解码是否成功
            if cv_image is None:
                self.get_logger().error("Failed to decode image")
                return
            
            self.image_queue.put(cv_image)
            self.image_count += 1
        
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def process_images(self):
        while rclpy.ok():
            if self.image_queue.qsize() >= 10:
                images = [self.image_queue.get() for _ in range(10)]
                self.convert_and_send_images(images)

    def convert_and_send_images(self, images):
        print("Converting images to VP9")
        
        try:
            output_file = "/tmp/output.webm"
            input_stream = ffmpeg.input('pipe:', format='image2pipe', framerate=1)
            output_stream = ffmpeg.output(input_stream, output_file, vcodec='libvpx-vp9', video_bitrate='1M')
            process = ffmpeg.run_async(output_stream, pipe_stdin=True, pipe_stdout=True, pipe_stderr=True)

            for image in images:
                _, encoded_image = cv2.imencode('.jpg', image)
                process.stdin.write(encoded_image.tobytes())
            
            process.stdin.close()
            process.wait()

            with open(output_file, 'rb') as f:
                print(f"Sending VP9 data, size: {os.path.getsize(output_file)} bytes")
                byte_data = f.read()
                byte_msg = ByteMultiArray()

                # 设置 MultiArrayLayout
                dim = [MultiArrayDimension(label="byte_array", size=len(byte_data), stride=1)]
                layout = MultiArrayLayout(dim=dim, data_offset=0)
                byte_msg.layout = layout

                byte_msg.data = list(byte_data)
                self.publisher_.publish(byte_msg)
                self.get_logger().info(f"Successfully sent VP9 data, size: {len(byte_data)} bytes")

        except Exception as e:
            self.get_logger().error(f"Error during conversion: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageToVP9Sender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
