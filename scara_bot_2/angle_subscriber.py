import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CoordSubscriber(Node):
    def __init__(self):
        super().__init__('coord_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'topic',
            self.callback_coord,
            10
        )
        self.subscription  # prevent unused variable warning
        
    def callback_coord(self, msg):
        self.get_logger().info(f"Received coord: {msg.data[0]}, {msg.data[1]} ")

def main():
    rclpy.init()
    node = CoordSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
