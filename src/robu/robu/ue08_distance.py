import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import random

class DistanceSensorNode(Node):
    def __init__(self,node_name="distance_sensor"):
        super().__init__(node_name)
        self._pub_distance = self.create_publisher(Float32,"distance",10)
        self.create_timer(0.5,self._timer_distance_cb)
    
    def _timer_distance_cb(self):
        val = Float32()
        val.data = random.uniform(0.0,1.0)

        self._pub_distance.publish(val)
        self.get_logger().info(f"Distance: {val.data}")

def main_distance_sensor():
    try:
        rclpy.init()
        node = DistanceSensorNode()
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        print("Sie haben STRG+C gedrückt!")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def main_obstacle_avoider():
    try:
        rclpy.init()
        node = ObstacleAvoidanceNode()
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        print("Sie haben STRG+C gedrückt!")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
class ObstacleAvoidanceNode(Node):
    def __init__(self,node_name="obstacle_avoidence"):
        super().__init__(node_name)
        self._sub_distance = self.create_subscription(Float32,"distance",self._sub_distance_cb,10)
        self._distance_min = 0.3
    def _sub_distance_cb(self,msg:Float32):
        if msg.data < self._distance_min:
            self.get_logger().warn("Kollision steht unmittelbar bevor")

if __name__ == "__main__":
    main_distance_sensor()