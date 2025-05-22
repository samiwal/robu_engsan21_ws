from rclpy.node import Node
import rclpy
import os
import select
import time
import sys
import tty
import termios
from std_msgs.msg import Int8

class IntPublisher(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        intpublisher = self.create_publisher(Int8,"motor_vlc",10)
        self._vel = Int8()
        try:
            while True:
                key = self.get_key()
                if key != '':
                    s = "String: " + key.replace(chr(0x1B), '^') + ", Code:"
                    for c in key:
                        s += " %d" % ord(c)
                    print(s)
                    if ord(key[0]) == 0x03:  # Ctrl+C
                        break
                    elif key == "\x1B[A":  # Pfeil nach oben
                        if self._vel.data < 125:
                            self._vel.data += 5
                            intpublisher.publish(self._vel)
                    elif key == "\x1B[B":  # Pfeil nach unten
                        if self._vel.data > -125:
                            self._vel.data -= 5
                            intpublisher.publish(self._vel)
        except Exception as e:
            print(e)
        

    def destroy_node(self):
        return super().destroy_node()
    
    def get_key(self):
        old_settings = termios.tcgetattr(sys.stdin)
        ts = time.time()
        key = ''
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key += os.read(sys.stdin.fileno(), 1).decode("utf-8")
                else:
                    break
            return key
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = IntPublisher("intpublisher")
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