#Exercise Title:    "LED-Strip"
#Name:              ?
#Group:             ?
#Class:             ?
#Date:              ?

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
# Antwort 1: 
# Frage 2 - Wofür wird die ROS_DOMAIN_ID verwendet, und was passiert, wenn du sie auf 0 setzt?
# Antwort 2: 
# Frage 3 - Warum haben Anwender, die eine virtuelle Maschine nutzen, Probleme damit, ROS-Nachrichten von Remote-Nodes anzuzeigen?
# Antwort 3: 
# Frage 4 - Wie kannst du über die CLI herausfinden, welche ROS-Nachrichten gerade verfügbar sind?
# Antwort 4: 
# Frage 5 - Wie lautet der CLI-Befehl, um die USER-LED zu schalten?
# Antwort 5: 
# Frage 6 - Wie lautet der CLI-Befehl, um die TEAM-LED zu schalten?
# Antwort 6: 
# Frage 7 - Was ist ein PWM-Signal, wofür wird es in unserem Beispiel verwendet, und wie bzw. von wem wird es erzeugt?
# Antwort 7: 
# Frage 8 - Was musst du tun, um deinen ROS-Node über einen ROS-Befehl in der CLI starten zu können? Wie könnte das Programm alternativ gestartet werden?
# Antwort 8: 
# Frage 9 - Welche LED-Farbe hat deine individuelle LED?
# Antwort 9: 
# Frage 10 - Welchen Datentyp müssen die ROS-Nachrichten mit den Topics "user" und "team" haben?
# Antwort 10: 


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
    pass


if __name__ == '__main__':
    main()