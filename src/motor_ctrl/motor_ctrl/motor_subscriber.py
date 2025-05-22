from rclpy.node import Node
import rclpy
from std_msgs.msg import Int8
from dynamixel_sdk import *

class MotorCtrl(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        #Motor Starten
        self.create_subscription(Int8,"motor_vlc",self._cb_sub_motorctrl,10)

    def _cb_sub_motorctrl(self,msg:Int8):
        print(f"Subscription succesfull. Motor Velocity:{msg.data}")
        #Motor übergeben
    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = MotorCtrl("motorctrl")
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Sie haben STRG+C gedrückt!")

    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info(f"Node {node.get_name()} wird beendet!")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()