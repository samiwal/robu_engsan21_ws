from math import sin ,radians
import signal
import sys
from rpi_ws281x import ws, Adafruit_NeoPixel, Color
import time
#https://github.com/rpi-ws281x/rpi-ws281x-python
#https://cdn.sparkfun.com/datasheets/BreakoutBoards/WS2812B.pdf
#installation der Bibliothek am Raspberry sudo pip install rpi_ws281x
give = sys.argv[1]

LED_COUNT = 3 #Anzahl der verwendeten LEDS
LED_PIN = 21 #=GPIO021 am Raspberry: Dieser Pin verfügt über PWM-Peripherie

LED_FREQUENZ_HZ = 800000 #Kommunikationszeit = 1.25 µs
LED_DMA = 10 #Direct Memory Access (=Hardware-Peripherie)
LED_BRIGHTNESS = 255 #Höchster wert bei 8-Bit

LED_INVERT = False
LED_CHANNEL = 0 #0 or 1
LED_STRIP = ws.WS2811_STRIP_GRB

ampel = Adafruit_NeoPixel(LED_COUNT,
                          LED_PIN,
                          LED_FREQUENZ_HZ,
                          LED_DMA,
                          LED_INVERT,
                          LED_BRIGHTNESS,
                          LED_CHANNEL,
                          LED_STRIP)
ampel.begin()
def turn_off_leds(sig,frame):
    print("Du hast Strg+C gedrückt")
    print("Die LEDs werden deaktiviert")
    for i in range(LED_COUNT):
        ampel.setPixelColor(i, Color(0,0,0))
    ampel.show()
    sys.exit(0)
#CallBack Funktion
signal.sinal(signal.SIGINT, turn_off_leds)
colors = [ Color(255,0,0),Color(0,255,0),Color(0,0,255)]
if give == "blink":
    while True:
        for i in range(LED_COUNT):
            ampel.setPixelColor(i,Color(255,255,0))
            time.sleep(0,2)
        for i in range(LED_COUNT):
            ampel.setPixelColor(i,Color(0,0,0))
            time.sleep(0,2)
if give >= 4:
    give -= 4
    ampel.setPixelColor(0,Color(255,0,255))
if give >= 2:
    give -= 2
    ampel.setPixelColor(1,Color(255,255,0))
if give >= 1:
    ampel.setPixelColor(2,Color(0,255,255))



