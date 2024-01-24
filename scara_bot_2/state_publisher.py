from math import pi
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class StatePublisher(Node):

    
    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        self.data = None

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'topic',
            self.callback_coord,
            10
        )
        self.subscription 

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        

    def callback_coord(self, msg):
        data = msg.data
        self.get_logger().info(f"Received coord: {msg.data[0]}, {msg.data[1]} ")
        self.data = data
        
def inki(x,y):
        l1 = 5
        l2 = 5
        theta2 = math.acos((x**2 + y**2 -l1**2-l2**2)/(2*l1*l2))
        theta1 = math.acos((x*(l1+l2*math.cos(theta2)) + y*(l2*math.sin(theta2)))/(x**2 + y**2))
        return theta1, theta2

def main():
    node = StatePublisher()
    
    while rclpy.ok():
        rclpy.spin_once(node)

        if node.data is not None:
            
            degree = pi / 180.0
    
            loop_rate = node.create_rate(10)

            x = node.data[0]
            y = node.data[1]


            # robot state
            angle1, angle2 = inki(x,y)
            a1 = 0.0
            a2 = 0.0
            joint_state = JointState()
            joint_state.header.frame_id=""
            joint_state.name = ["Link_1_joint","Link_2_joint"]

   
            try:
                while True:
                    
                    rclpy.spin_once(node)
                    now = node.get_clock().now()
                
                    print("\n\n")
                    print(joint_state)
                    print("\n\n")


                    joint_state.header.stamp = now.to_msg()
                    joint_state.position = [a1,a2]
                    
                    node.joint_pub.publish(joint_state)
                    if a1 >= angle1:
                        a1 = angle1 
                        if a2>=angle2:
                            a2 = angle2
                        else:
                            a2+=degree
                    elif a2 >= angle2:
                        a2 = angle2
                        if a1>=angle1:
                            a1 = angle1
                        else:
                            a1+=degree
                    else:
                        a1 = a1+degree
                        a2 = a2+degree
            

                    loop_rate.sleep()
                
                
                

            except KeyboardInterrupt:
                pass
            
            node.data = None
    
    node.destroy_node()
    rclpy.shutdown()
                

            
    

if __name__ == '__main__':
    main()