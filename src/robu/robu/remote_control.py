#Exercise Title:    Remote Control for the TurtleBot3 Burger
#Group:             ?
#Class:             ?
#Date:              ?

import rclpy
import os
import select
import sys
import time


from geometry_msgs.msg import Twist
import rclpy.logging
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

msg = """
Excercise:  ?
Group:      ?
Class:      ?
Date:       ?
"""

e = """
Communications Failed
"""

#Physikalische Grenzen des Roboters
MAX_LIN_VEL = 0.22          #m/s
MAX_ANG_VEL = 2.84          #rad/s

LIN_VEL_STEP_SIZE = 0.01    #m/s
ANG_VEL_STEP_SIZE = 0.1     #rad/s

def get_key():
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
    
    rclpy.init()

    qos = QoSProfile(depth=10)
    
    node = rclpy.create_node('remotectrl')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    vel = Twist()

    try:
        while(1):
            key = get_key()
            if key != '':
                str = "String: " + key.replace(chr(0x1B), '^') + ", Code:"
                for c in key:
                    str += " %d" % (ord(c))
                print(str)

                if ord(key[0])== 0x03:
                    break
                elif key == "\x1B":
                    vel.linear.x = 0.0
                    vel.linear.y = 0.0
                    vel.linear.z = 0.0

                    vel.angular.x = 0.0
                    vel.angular.y = 0.0
                    vel.angular.z = 0.0
                elif key == "\x1B[A": #Pfeiltaste rauf
                    vel.linear.x += LIN_VEL_STEP_SIZE
                    if vel.linear.x > MAX_LIN_VEL:
                        vel.linear.x = MAX_LIN_VEL
                        node.get_logger().info("Maximale-geschwindigkeit-erreicht")
                        print("max v erreicht")
                pub.publish(vel)

    except Exception as e:
        print(e)

    finally:
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        pub.publish(vel)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()