import rclpy
import math
from turtlesim.msg import Pose
from rclpy.node import Node

class SimpleKinematics(Node):
    def __init__(self,node_name:str):
        super().__init__(node_name)
        self.create_subscription(Pose,"/turtle1/pose",self._sub_turtle1_pose_cb,10)
        self.create_subscription(Pose,"/turtle2/pose",self._sub_turtle2_pose_cb,10)

        self._turtle1_pose = Pose()
        self._turtle2_pose = Pose()
        self._tx = 0.0
        self._ty = 0.0
    
    def _rechnen_translation(self):
        self._tx = self._turtle1_pose.position.x - self._turtle2_pose.position.x
        self._ty = self._turtle1_pose.position.y - self._turtle2_pose.position.y
    
    def _sub_turtle1_pose_cb(self,msg:Pose):
        self._turtle1_pose = msg

    def _sub_turtle2_pose_cb(self,msg:Pose):
        self._turtle2_pose = msg
        self._rechnen_translation()

        self.get_logger().info(f"Translation-Vector: Tx={self._tx}, Ty={self._ty}")
        self._theta = self._turtle1_pose.theta - self._turtle2_pose.theta
        rotation_matrix = [[math.cos(self._theta), -math.sin(self._theta)],
                           [math.sin(self._theta), math.cos(self._theta)]]
        
        self.get_logger().info(f"Theta: {self.theta},Rotation Matrix: {rotation_matrix}")

def main():
    try:
        rclpy.init()
        node = SimpleKinematics("simple_kinematics")
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        print("Sie haben STRG+C gedrückt!")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
