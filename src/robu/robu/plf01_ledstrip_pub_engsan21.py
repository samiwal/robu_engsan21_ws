#Exercise Title:    "LED-Strip"
#Name:              "engsan21"
#Group:             "4"
#Class:             "4BHME"
#Date:              "30.1.2025"

import rclpy
import rclpy.logging
from rclpy.qos import QoSProfile

import os
import select
import sys
import time

import termios
import tty

# Fragen:
#--------------------------------
# Frage 1 - Wie kannst du die ROS_DOMAIN_ID in der CLI auf 0 setzen?
# Antwort 1: export ROS_DOMAIN_ID=0
# Frage 2 - Wofür wird die ROS_DOMAIN_ID verwendet, und was passiert, wenn du sie auf 0 setzt?
# Antwort 2: Um eine adresse im Netzwerk direkt anzusprechen. Wenn sie 0 ist, wird die standartadresse verwendet
# Frage 3 - Warum haben Anwender, die eine virtuelle Maschine nutzen, Probleme damit, ROS-Nachrichten von Remote-Nodes anzuzeigen?
# Antwort 3: Weil die Virtuelle maschine ein eigenes netzwerk hat(ist mit "lan" mit dem computer verbunden)mashchine->netzwerk->bridged(braucht ststische ip adresse)
# Frage 4 - Wie kannst du über die CLI herausfinden, welche ROS-Nachrichten gerade verfügbar sind?
# Antwort 4: ros2 topic list
# Frage 5 - Wie lautet der CLI-Befehl, um die USER-LED zu schalten?
# Antwort 5: ros2 topic pub /user std_msgs/msg/String "{data: engsan21}"
# Frage 6 - Wie lautet der CLI-Befehl, um die TEAM-LED zu schalten?
# Antwort 6: ros2 topic pub /team4 std_msgs/msg/UInt8MultiArray "{data: [4,255,255,255,255]}"
# Frage 7 - Was ist ein PWM-Signal, wofür wird es in unserem Beispiel verwendet, und wie bzw. von wem wird es erzeugt?
# Antwort 7: Ein rechtecksignal, dessen 1 und 0 zeit moduliert werden können. Die periodendauer bleibt dabei gleich.
                #Um die LEDS anzusteuern. Das Signal kann vonn der LED "verarbeitet" werden
                #Es wird von einem Quartz auf einem raspberry erzeugt
# Frage 8 - Was musst du tun, um deinen ROS-Node über einen ROS-Befehl in der CLI starten zu können? Wie könnte das Programm alternativ gestartet werden?
# Antwort 8: ros2 run turtlesim turtlesim_node. Es sollte möglich sein, das programm über die taskleiste zu starten
# Frage 9 - Welche LED-Farbe hat deine individuelle LED?
# Antwort 9: Grün
# Frage 10 - Welchen Datentyp müssen die ROS-Nachrichten mit den Topics "user" und "team" haben?
# Antwort 10: user: string(kürzel) team: UInt8MultiArray(liste aus mehreren bytes)


# Programmieraufgabe 1: 
# --------------------------------
# Programmiere ein Programm zur Steuerung von RGBW-LEDs und speichere diese in dein robu Paket. Ein Subscriber-Node wurde bereits programmiert 
# und auf dem Raspberry gestartet 
# (siehe: https://github.com/mlieschnegg/robu_rpi_ws/blob/main/src/robu_rpi_examples/robu_rpi_examples/ledstrip_sub.py, von LI/SH). 
# Die Topics dieser ROS-Nachrichten lauten "user" sowie "team1" bis "team6".
# 
# Am LED-Streifen sind folgende LEDs reserviert:
#    - Eine individuelle LED (Topic "user") für dich. 
#    - Fünf LEDs für deine ROBU-Gruppe (Topics "team1" bis "team6").
#
# Implementiere:
#    - Je einen Publisher für deine individuelle LED sowie die LEDs deines Teams.
# 
# Anforderungen:
#    - Wenn du die Taste 'u' drückst, soll die individuelle LED ein- bzw. ausgeschaltet werden.
#    - Wenn du die Taste 't' drückst, sollen die LEDs deines Teams mit einer bestimmten Farbe (freie Wahl!) konfiguriert werden.
# 
# Hinweis: 
#    - Der grundlegende Aufbau deines Programms entspricht dem Programm aus Übung 6.
#    - Studiere den Code des Subscribers und finde heraus welchen Datentyp und Nachrichteninhalt 
#      du beim Puschlisher zum Schalten der LEDs verwenden musst.

#Abgabe
#------------------------
#Drucke diese Dokument aus (doppelseitig) und gib es bei LI/SH ab
from std_msgs.msg import String, UInt8MultiArray

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
    
    node = rclpy.create_node('ledstrip')
    pubu = node.create_publisher(String, 'user', qos)
    pubt = node.create_publisher(UInt8MultiArray, 'team4', qos)

    teammsg = UInt8MultiArray()
    usermsg = String()
    usermsg.data = 'engsan21'
    i = 0

    try:
        while(1):
            key = get_key()
            if key != '':
                str = "String: " + key.replace(chr(0x1B), '^') + ", Code:"
                for c in key:
                    str += " %d" % (ord(c))
                print(str)
                if key == "\x03":
                    break
                elif key == 'u':
                    pubu.publish(usermsg)
                elif key == 't':
                    if(i==0):
                        teammsg.data = [4,255,255,255,255]
                        pubt.publish(teammsg)
                        i=1
                    if(i==1):
                        teammsg.data = [4,0,0,0,0]
                        pubt.publish(teammsg)
                        i=0

    except Exception as e:
        print(e)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()