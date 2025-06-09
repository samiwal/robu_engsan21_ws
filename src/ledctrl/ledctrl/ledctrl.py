
#from uebungen.rpi_utils import is_raspberry_pi
import sys
import signal

import numpy
from rclpy.node import Node
from rclpy.timer import Timer
import rclpy
import yaml
from rclpy.parameter import Parameter

from ledctrl_interfaces.srv import BacklightValue, CameraCmd, ColorValues
from std_srvs.srv import Trigger
from std_msgs.msg import UInt8MultiArray

from rpi_ws281x import ws, Adafruit_NeoPixel, Color

#https://github.com/rpi-ws281x/rpi-ws281x-python
#https://cdn.sparkfun.com/datasheets/BreakoutBoards/WS2812B.pdf
#Installation der Bibliothek am Raspberry: sudo pip install rpi_ws281x

#ros2 service call /camera_bl leddisplay_interfaces/srv/LEDCamera "{camera: left, brightness_percent: 50}"

#Terminal 1:
# colcon build
# source install/setup.bash
# ros2 launch ledctrl ledctrl.launch.py 
#
#Terminal 2:
# source install/setup.bash
# Lässt die LEDs über der linken Kamera (side = 0) als Lauflicht blinken (cmd = 'turn')
# ros2 service call /camera_cmd ledctrl_interfaces/srv/CameraCmd "{cmd: 'turn', side: 0, color_on: [255, 0, 0, 0], color_off: [0, 255, 0, 0], duration_on: 0.2, duration_off: 0.2, repetitions: 5}"
# Lässt die LEDs über der linken Kamera (side = 0) blinken (einmal an, einmal aus) (cmd = 'warn')
# ros2 service call /camera_cmd ledctrl_interfaces/srv/CameraCmd "{cmd: 'detect', side: 0, color_on: [255, 0, 0, 0], color_off: [0, 255, 0, 0], duration_on: 0.2, duration_off: 0.2, repetitions: 5}"
# Stellt die Farbe der LED an der Posistion 0 auf rot (255, 0, 0, 0)
# ros2 topic pub -1 /ledctrl std_msgs/msg/UInt8MultiArray "{data: [0, 255, 0, 0, 0]}"

from pathlib import Path
import re

from enum import Enum

class LEDCtrlCmd(Enum):
    CAMERA_CMD_TURN = "turn"
    CAMERA_CMD_WARN = "warn"
    CAMERA_CMD_DETECT = "detect"

class LEDCtrlSide(Enum):
    LEFT = 0
    RIGHT = 1
    BOTTOM = 2

LED_CONF_LENGTH = 5  # pos, r, g, b, w

def is_raspberry_pi():
    CPUINFO_PATH = Path("/proc/cpuinfo")
    if not CPUINFO_PATH.exists():
        return False
    with open(CPUINFO_PATH) as f:
        cpuinfo = f.read()
    return re.search(r"^Model\s*:\s*Raspberry Pi", cpuinfo, flags=re.M) is not None

def new_led_strip(numleds:int) -> Adafruit_NeoPixel:
    if not is_raspberry_pi():
        return None
    
    LED_PIN = 21            #=GPIO21 am Raspberry: 
                            #Dieser Pin verfügt über PWM-Peripherie und PCM-Peripherie
    LED_FREUQ_HZ = 800000   #Kommunikationsfrequenz 800kHz -> T=1.25 µs
    LED_DMA = 10            #Direct Memory Access (=Hardware-Peripherie)
    LED_BRIGHTNESS = 255    #Höchster Wert bei 8-Bit = 2^8 = 256 -> 0 bis 255

    LED_INVERT = False      #LEDs sind direkt mit dem Raspberry verbunden (kein Transistor)
    LED_CHANNEL = 0         #0 or 1 -> 0 für PWM
    LED_STRIP = ws.WS2811_STRIP_GRB

    return  Adafruit_NeoPixel(numleds,
                              LED_PIN,
                              LED_FREUQ_HZ,
                              LED_DMA,
                              LED_INVERT,
                              LED_BRIGHTNESS,
                              LED_CHANNEL,
                              LED_STRIP)

class LEDCtrl(Node):
    _camera_cmd_request:CameraCmd.Request=None
    _timer_camera_cmd:Timer=None
    _hw_led_strip:Adafruit_NeoPixel=None

    _camera_cmd_on:bool = True

    def __init__(self, node_name: str):
        super().__init__(node_name)
        #-> request cam1/cam2, level (0-100%) -> response: [id] [wrgb-value]
        self._srv_set_backlight = self.create_service(BacklightValue, "set_backlight", self._srv_set_backlight_cb)
        self._srv_camera_cmd = self.create_service(CameraCmd, "camera_cmd", self._srv_camera_cmd_cb)
        self._srv_save_parameters = self.create_service(Trigger, "save_parameters", self._srv_save_parameters_cb)
        self._srv_read_colors = self.create_service(ColorValues, "read_colors", self._srv_read_colors_cb)

        self.create_subscription(UInt8MultiArray, "ledctrl", self._sub_ledctrl_cb, 10)

        self.declare_parameter("colors", [0])
        self.set_parameters([Parameter("colors", rclpy.Parameter.Type.INTEGER_ARRAY, [])])

        self.declare_parameters("", [("turn_signal_interval", 0.5),
                                     ("camera_cmd_duration_on", 0.2),
                                     ("camera_cmd_duration_off", 0.2),
                                     ("camera_cmd_repetitions", 5),
                                     ("camera_bl_color", [0, 0, 0, 255]),
                                     ("colorsensor_bl_color", [0, 0, 0, 255]),
                                     ("turn_signal_color_on", [255, 0, 0, 0]),
                                     ("turn_signal_color_off", [255, 255, 0, 0]),
                                     ("detect_color_on", [0, 255, 0, 0]),
                                     ("detect_color_off", [0, 0, 0, 0]),
                                     ("default_color", [0, 0, 0, 0])
                                     ])
        
        self._cam_left_pos = list(range(22,30,1))
        self._cam_right_pos = list(range(14,22,1))
        self._colorsensor_pos = [12,13]
        self._chassis_pos = list(range(0,14,1))

        numleds = max(
            self._cam_left_pos + 
            self._cam_right_pos +
            self._colorsensor_pos +
            self._chassis_pos) + 1

        #pos = range(min(pos), max(pos)+1)        
        self._led_values = numleds*[[0,0,0,0]]
        
        for pos in self._cam_left_pos:
            self._led_values[pos] = self.get_parameter("camera_bl_color").value
        for pos in self._cam_right_pos:
            self._led_values[pos] = self.get_parameter("camera_bl_color").value
        for pos in self._colorsensor_pos:
            self._led_values[pos] = self.get_parameter("colorsensor_bl_color").value
        for pos in self._chassis_pos:
            self._led_values[pos] = self.get_parameter("default_color").value

        self._load_config(self.get_parameter("colors").value)

        self._is_rpi = is_raspberry_pi()
        self._hw_led_strip = new_led_strip(numleds)
        self._hw_led_strip_begin()
        self._hw_led_strip_update()

        self._turn_signal_pos = 0

        # self.create_service("battery") -> level (0-100%) -> response: [id] [wrgb-value]

    def _srv_set_backlight_cb(self, request: BacklightValue.Request, response: BacklightValue.Response):
        percent = min(100, int(request.brightness_percent))
        response.status = True

        if request.side not in (side.value for side in LEDCtrlSide):
            self.get_logger().warn(f"Invalid side {request.side}!")
            response.status = False
            return response
        
        match LEDCtrlSide(request.side):
            case LEDCtrlSide.LEFT:
                max_vals = self.get_parameter("camera_bl_color").value
                self.get_logger().info(f"Setting Camera Left backlight to {percent}%, max values: {max_vals}")
                new_value = [int((max_vals[i] * percent) / 100) for i in range(4)]
                for pos in self._cam_left_pos:
                    self._led_values[pos] = new_value
                self.get_logger().info(logstr)

            case LEDCtrlSide.RIGHT:
                max_vals = self.get_parameter("camera_bl_color").value
                new_value = [int((max_vals[i] * percent) / 100) for i in range(4)]
                for pos in self._cam_right_pos:
                    self._led_values[pos] = new_value
                logstr = "Camera Right -> color: [" + ", ".join(f"{v:03d}" for v in new_value) + "]"
                self.get_logger().info(logstr)
            case LEDCtrlSide.BOTTOM:
                max_vals = self.get_parameter("colorsensor_bl_color").value
                new_value = [int((max_vals[i] * percent) / 100) for i in range(4)]
                for pos in self._colorsensor_pos:
                    self._led_values[pos] = new_value
                logstr = "Color Sensor -> color: [" + ", ".join(f"{v:03d}" for v in new_value) + "]"
                self.get_logger().info(logstr)
            case _:
                self.get_logger().warn(f"Invalid service command {request.camera}!")
                response.status = False
        self._hw_led_strip_update()
        return response
    
    def _load_config(self, data:list):
        if len(data) % LED_CONF_LENGTH != 0:
            raise ValueError("LED configuration data length is not a multiple of LED_CONF_LENGTH")
        
        for i in range(len(data)//LED_CONF_LENGTH):
            pos = data[i*LED_CONF_LENGTH+0]
            r = data[i*LED_CONF_LENGTH+1]
            g = data[i*LED_CONF_LENGTH+2]
            b = data[i*LED_CONF_LENGTH+3]
            w = data[i*LED_CONF_LENGTH+4]
            self._led_values[pos] = [r, g, b, w]
    
    def _sub_ledctrl_cb(self, msg: UInt8MultiArray):
        self.get_logger().info(f"Nachricht für LightCtrl empfangen: {msg.data}")
        self._load_config(msg.data)
        self._hw_led_strip_update()
        
    def _srv_camera_cmd_cb(self, request:CameraCmd.Request, response:CameraCmd.Response):
        if request.cmd in (cmd.value for cmd in LEDCtrlCmd):

            if request.duration_on < 0.0 or request.duration_off < 0.0 or request.repetitions < 0:
                raise ValueError("Duration and repetitions must be non-negative")
            
            if request.duration_on == 0.0:
                request.duration_on = float(self.get_parameter("camera_cmd_duration_on").value)
            if request.duration_off == 0.0:
                request.duration_off = float(self.get_parameter("camera_cmd_duration_off").value)
            if request.repetitions == 0:
                request.repetitions = int(self.get_parameter("camera_cmd_repetitions").value)

            if not request.color_on: # is numpy.array("b", []):
                if request.cmd == LEDCtrlCmd.CAMERA_CMD_DETECT:
                    request.color_on = bytes(self.get_parameter("detect_color_on").value)
                else:
                    request.color_on = bytes(self.get_parameter("turn_signal_color_on").value)
            if not request.color_off: # is numpy.array("b", []):
                if request.cmd == LEDCtrlCmd.CAMERA_CMD_DETECT:
                    request.color_off = bytes(self.get_parameter("detect_color_off").value)
                else:
                    request.color_off = bytes(self.get_parameter("turn_signal_color_off").value)

            if request.cmd != LEDCtrlCmd.CAMERA_CMD_TURN:
                request.repetitions *= 2  # because we toggle on/off
            
            self._cancel_camera_cmd()

            self._camera_cmd_request = request
            self._led_values_backup = self._led_values.copy()  # Copy current LED values    

            self._camera_cmd_on = True
            self._timer_camera_cmd = self.create_timer(request.duration_on, self._timer_camera_cmd_cb)
            self.get_logger().info("Camera Command - Received")
            self._timer_camera_cmd_cb()
            response.status = True
        else:
            self.get_logger().warn(f"Camera Command - INVALID Command {request.cmd}")
            response.status = False

        return response

    def _cancel_camera_cmd(self):
        if self._timer_camera_cmd is not None:
            self._timer_camera_cmd.destroy()
            self._timer_camera_cmd = None
            self.get_logger().info("Camera Command - Canceled")
            self._led_values = self._led_values_backup.copy()  # Restore LED values
            
    def _get_color_values(self):
        config = []
        for pos, color in enumerate(self._led_values):
            config += [pos] + color
        return config

    def _srv_save_parameters_cb(self, request:Trigger.Request, response:Trigger.Response):
        response.success = True
        response.message = "LED configuration saved successfully."
        
        config = self._get_color_values()

        self.set_parameters([
            Parameter("colors", rclpy.Parameter.Type.INTEGER_ARRAY, config)
        ])

        # Jetzt alle Parameter sammeln
        params_dict = {}
        for param in self._parameters.values():
            if param.name not in ("use_sim_time",):  # Beispiel: Systemparameter ignorieren
                params_dict[param.name] = param.value

        # Im YAML-Format speichern
        try:
            with open("ledctrl.yaml", "w") as f:
                yaml.dump({self.get_name(): {"ros__parameters": params_dict}}, f, default_flow_style=False)
            response.message = "LED configuration dumped to ledctrl.yaml successfully."
        except Exception as e:
            response.message = f"Failed to save parameters as YAML: {e}"
            self.get_logger().error(response.message)

        return response
        
    def _srv_read_colors_cb(self, request:ColorValues.Request, response:ColorValues.Response):
        response.status = True
        response.colors = self._get_color_values()
        self.get_logger().info(f"Read colors: {response.colors}")
        return response
    
    def print_turn_signal(self, color_on:list, color_off:list):
        str_turn_signal = ""
        #self.get_logger().info(f"color_on: {color_on}, color_off: {color_off}")
        for pos in sorted(self._cam_left_pos):
            r, g, b, w = self._led_values[pos]
            if self._led_values[pos] == color_on:
                str_turn_signal += f"\033[38;2;{r};{g};{b}m<\033[0m"
            else:
                str_turn_signal += f"\033[38;2;{r};{g};{b}mo\033[0m"
        
        str_turn_signal += 20*" "
        for pos in sorted(self._cam_right_pos):
            r, g, b, w = self._led_values[pos]
            if self._led_values[pos] == color_on:
                str_turn_signal += f"\033[38;2;{r};{g};{b}m>\033[0m"
            else:
                str_turn_signal += f"\033[38;2;{r};{g};{b}mo\033[0m"

        self.get_logger().info(str_turn_signal)
        # print(str_turn_signal)

    def _update_turn_signal(self, side, color_on:list, color_off:list):
        if side == LEDCtrlSide.LEFT:
            leds_pos = self._cam_left_pos
        elif side == LEDCtrlSide.RIGHT:
            leds_pos = self._cam_right_pos
        else: 
            return
        
        for pos in leds_pos:
            self._led_values[pos] = color_off
        pos = leds_pos[self._turn_signal_pos]
        self._led_values[pos] = color_on

    def _update_warn_signal(self, color_on:list, color_off:list):
        for pos in self._cam_left_pos:
            if self._led_values[pos] == color_off:
                self._led_values[pos] = color_on
            else:
                self._led_values[pos] = color_off
        
        for pos in self._cam_right_pos:
            if self._led_values[pos] == color_off:
                self._led_values[pos] = color_on
            else:
                self._led_values[pos] = color_off

    def _update_detect_signal(self, side, color_on:list, color_off:list):
        if side == LEDCtrlSide.LEFT:
            leds_pos = self._cam_left_pos
        elif side == LEDCtrlSide.RIGHT:
            leds_pos = self._cam_right_pos
        else: 
            return

        for pos in leds_pos:
            if self._led_values[pos] == color_off:
                self._led_values[pos] = color_on
            else:
                self._led_values[pos] = color_off

    def _timer_camera_cmd_cb(self):
        rq = self._camera_cmd_request
        cmd = LEDCtrlCmd(rq.cmd)
        side = LEDCtrlSide(rq.side)
        # self.get_logger().info(f"Camera Command - {rq.cmd} on: {rq.duration_on:.2f}s, off: {rq.duration_off:.2f}s, repetitions: {rq.repetitions}, side: {rq.side}, color_on: {rq.color_on}, color_off: {rq.color_off}")
        if rq.repetitions > 0:
            if cmd == LEDCtrlCmd.CAMERA_CMD_TURN:
                numleds = len(self._cam_left_pos)
                self._update_turn_signal(side, rq.color_on.tolist(), rq.color_off.tolist())
                self._turn_signal_pos += 1
                self._turn_signal_pos = self._turn_signal_pos % numleds
            elif cmd == LEDCtrlCmd.CAMERA_CMD_WARN:
                self._update_warn_signal(rq.color_on.tolist(), rq.color_off.tolist())
            elif cmd == LEDCtrlCmd.CAMERA_CMD_DETECT:                    
                self._update_detect_signal(side, rq.color_on.tolist(), rq.color_off.tolist())

            self._camera_cmd_on = not self._camera_cmd_on
            if (rq.duration_on != rq.duration_off):
                self._timer_camera_cmd.destroy()
                self._timer_camera_cmd = self.create_timer(rq.duration_on if self._camera_cmd_on else rq.duration_off, self._timer_camera_cmd_cb)
        else:
            self._led_values = self._led_values_backup.copy()  # Restore LED values
            self._turn_signal_pos = 0
            self._timer_camera_cmd.destroy()
            self._timer_camera_cmd = None
        
        rq.repetitions -= 1
        self.print_turn_signal(rq.color_on.tolist(), rq.color_off.tolist())
        if rq.repetitions < 0:
            self.get_logger().info("Camera Command - OFF")
            
        self._hw_led_strip_update()

    def _hw_led_strip_begin(self):
        if self._is_rpi:
            self._hw_led_strip.begin()
            self._hw_led_strip_update()

    def _hw_led_strip_update(self):
        if self._is_rpi:
            for i, (r,g,b,w) in enumerate(self._led_values):
                self._hw_led_strip.setPixelColorRGB(i,r, g, b, w)
            self._hw_led_strip.show()

    def destroy_node(self):
        # Schalte alle LEDs aus, bevor der Node beendet wird
        for i in range(self._hw_led_strip.numPixels()):
            self._hw_led_strip.setPixelColor(i, Color(0, 0, 0, 0))
        self._hw_led_strip.show()
        return super().destroy_node()

def turn_off_leds(sig, frame):
    global node
    print("Du hast STRG+C gedrückt!")
    print("Die LEDs werden deaktiviert!")
    del node
    sys.exit(0)


def main(args=None):
    node = None

    try:
        rclpy.init(args=args)
        temp_node = rclpy.create_node('__check_node__')
        nodes_info = temp_node.get_node_names_and_namespaces()
        running_node_names = [name for name, ns in nodes_info]

        target_name = 'ledctrl'
        temp_node.destroy_node()

        if target_name not in running_node_names:
            try:
                node = LEDCtrl(target_name)
            except Exception as e:
                print(f"Fehler beim Erstellen des Nodes: {e}")
                return
            
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt erkannt, beende Node.")
    finally:
        if node is not None:
            node.destroy_node()
            # if rclpy.ok():
            #     node.get_logger().info(f"Node {node.get_name()} wurde beendet!")
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
