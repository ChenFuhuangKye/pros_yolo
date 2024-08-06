import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        my_param = self.declare_parameter('param_name', 'default_value')
        
        test_out = my_param.value + '_yolo'
        self.get_logger().info(f'Parameter value: {my_param.value}')
        self.get_logger().info(f'Test output: {test_out}')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
