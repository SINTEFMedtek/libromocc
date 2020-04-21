from pyromocc import Robot
import numpy as np
from time import sleep

robot = Robot(ip="192.168.153.131", port=30003, manipulator="UR10")
robot.connect()

sleep(1.0)
print(robot.joint_config)

sleep(1.0)
print(robot.joint_config)

robot.movej([0.0, -np.pi/2, -np.pi/2, -np.pi/2, 0, 0], 50, 100)

sleep(1.0)
print(robot.joint_config)