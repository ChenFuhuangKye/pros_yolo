import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from car_models import *
import cv2
import numpy as np
import os
import orjson


class CarMove(Node):
        def __init__(self):
            super().__init__('car_move')
            
            self.subscription_bbox = self.create_subscription(
                String,
                '/camera/bbox',
                self.listener_callback,
                10)
            self.subscription_bbox  # prevent unused variable warning
            self.publisher = self.create_publisher(
                String,
                '/wheel_speed',
                10)
            
            self._vel1 = 10 # left wheel speed rad/s
            self._vel2 = 0 # right wheel speed rad/s
        def listener_callback(self, msg):
            msg_str = msg.data
            print('I heard: "%s"' % msg_str)
            # self._pub_control()
        
        def _pub_control(self):
            # Generate a random control signal
            control_signal = {
                "type": str(DeviceDataTypeEnum.car_control),
                "data": dict(CarCControl(
                    target_vel=[self._vel1, self._vel2]
                ))
            }
            # Convert the control signal to a JSON string
            control_msg = String()
            control_msg.data = orjson.dumps(control_signal).decode()

            # Publish the control signal
            self.publisher.publish(control_msg)

            # Convert the control signal to a JSON string

            self.get_logger().info(f'publish {control_msg}')



def main(args=None):
    rclpy.init(args=args)

    car_move = CarMove()

    rclpy.spin(car_move)

    car_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()