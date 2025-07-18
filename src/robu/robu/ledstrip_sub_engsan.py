import rclpy
from std_msgs.msg import String,UInt8MultiArray

def main():
    try:
        rclpy.init()
        node_ledstrip = rclpy.create_node("ledstrip_sub")
        node_ledstrip.create_subscription(String,"user",callback_user,10)
        node_ledstrip.create_subscription(UInt8MultiArray,"team4",callback_team,10)

        rclpy.spin(node_ledstrip)#Node Ausführen,Programm bleibt hängen
    except KeyboardInterrupt as e:
        print("Sia haben Strg+C gedrückt!")
    finally:
        node_ledstrip.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            pass
def callback_user(msg:String):
    global node_ledstrip
    print(f"User: {msg.data}")
    node_ledstrip.get_logger().info(f"User: {msg.data}")
def callback_team(msg:UInt8MultiArray):
    global node_ledstrip
    print(f"Team_LEDs: {msg.data}")
    node_ledstrip.get_logger().info(f"Team_LEDs: {msg.data}")

if __name__=="__main__":
    main()