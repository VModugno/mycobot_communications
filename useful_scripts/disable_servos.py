import pymycobot
from pymycobot.mycobot import MyCobot
from pymycobot.error import MyCobotDataException

import time

def main():
    port = "/dev/ttyAMA0"
    baud = 1000000
    slp_time = 60
    mc = MyCobot(port, baud)
    connected = mc.is_controller_connected()
    if not connected:
        raise RuntimeError
    mc.release_all_servos()

    time.sleep(slp_time)

    print("starting to power on servos")
    time.sleep(15)
    [mc.focus_servo(servo_id) for servo_id in range(1, 7)]