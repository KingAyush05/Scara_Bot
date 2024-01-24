from math import pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        angle = 0.0
        joint_state = JointState()
        joint_state.header.frame_id=""
        joint_state.name = ["Link_1_joint","Link_2_joint"]

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                now = self.get_clock().now()

                print("\n\n")
                print(joint_state)
                print("\n\n")

                joint_state.header.stamp = now.to_msg()
                joint_state.position = [angle,angle]

                self.joint_pub.publish(joint_state)
                
                angle += degree

                loop_rate.sleep()

        except KeyboardInterrupt:
            pass



def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()