import rclpy
from math import pi
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class CoordPublisher(Node):

    def __init__(self):
        super().__init__('coord_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_coord)


    def publish_coord(self):
        x = float(input("Enter x: "))
        y = float(input("Enter y: "))

        msg = Float64MultiArray()
        msg.data = [x, y]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published values: {x}, {y} ")
        


def main(args=None):
    rclpy.init(args=args)

    coord_publisher = CoordPublisher()

    rclpy.spin(coord_publisher)
    rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #minimal_publisher.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()