from math import pi
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion

class StatePublisher(Node):

    
    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        
        self.trajectory_points = [
            [10.0, 0.0],   # Joint positions for point 1
            [8.365, 4.8296],   # Joint positions for point 2
            [5.6242, 7.3295],   # Joint positions for point 3
            [2.241, 8.365],   # Joint positions for point 4 (looping back to start)
        ]
        self.angles = []
        self.current_trajectory_point = 0

def inki(x,y):
        l1 = 5
        l2 = 5
        theta2 = math.acos((x**2 + y**2 -l1**2-l2**2)/(2*l1*l2))
        theta1 = math.acos((x*(l1+l2*math.cos(theta2)) + y*(l2*math.sin(theta2)))/(x**2 + y**2))
        return theta1, theta2

def main():
    
    node = StatePublisher()
    
    while rclpy.ok():

        
            
        degree = pi / 180.0
    
        loop_rate = node.create_rate(10)

        for i in range(len(node.trajectory_points)):
            x = node.trajectory_points[i][0]
            y = node.trajectory_points[i][1]
            angle1, angle2 = inki(x,y)
            node.angles.append([angle1, angle2])
                


        # robot state
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
                joint_state.position = node.angles[node.current_trajectory_point]
                    
                node.current_trajectory_point = (node.current_trajectory_point + 1) % len(node.trajectory_points)
                    
                node.joint_pub.publish(joint_state)

                loop_rate.sleep()
                
                
                

        except KeyboardInterrupt:
            pass
            
            
    
    node.destroy_node()
    rclpy.shutdown()
                

if __name__ == '__main__':
    main()