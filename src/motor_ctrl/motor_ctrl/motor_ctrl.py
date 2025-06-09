import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import numpy as np
from motor_ctrl.dynamixel import Dynamixel, OperatingMode

class Ros2VelocityControl(Node):
    def __init__(self):
        super().__init__('dynamixel_velocity_node')

        self.subscription = self.create_subscription(Int8, 'motor_vlc', self.callback, 10)

        # Dynamixel Setup
        self.motor_id = 4
        self.dxl = Dynamixel.Config(
            baudrate=57600,
            device_name='/dev/ttyAMA1'  # dein Port laut Wrapper
        ).instantiate()
        self.dxl._disable_torque(self.motor_id)                 
        self.dxl.set_operating_mode(self.motor_id, OperatingMode.VELOCITY)
        self.dxl._enable_torque(self.motor_id)

        self.max_input = 125
        self.max_speed = int(self.dxl.get_velocity_limit(self.motor_id) * 0.9)  # 90 % sicherheitsgrenze

        self.get_logger().info(f"Motor ID {self.motor_id} bereit mit max Speed {self.max_speed}")

    def callback(self, msg: Int8):
        input_val = max(-self.max_input, min(self.max_input, msg.data))
        velocity = int(input_val / self.max_input * self.max_speed)

        self.dxl.set_goal_velocity(self.motor_id, velocity)
        self.get_logger().info(f"Eingang: {input_val}, gesetzt: {velocity}")

    def destroy_node(self):
        # Motor stoppen beim Beenden
        self.dxl.set_goal_velocity(self.motor_id, 0)
        self.dxl._disable_torque(self.motor_id)
        self.dxl.disconnect()
        super().destroy_node()

def main():
    rclpy.init()
    node = Ros2VelocityControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Node wird beendet...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()