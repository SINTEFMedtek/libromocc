import time

from pyromocc.tools import SpaceMouse
from pyromocc import Robot

robot = Robot(ip="localhost", port=30003)
robot.connect()

space_mouse = SpaceMouse()
space_mouse.connect()


while True:
    raw_state = space_mouse.get_operational_config(calibrated=True)
    target_speed = [raw_state[0]*100, raw_state[1]*100, raw_state[2]*100,
                    raw_state[3]*100, raw_state[4]*100, raw_state[5]*100]
    robot.speedl(target_speed, 100, 0.5)
    time.sleep(0.01)
