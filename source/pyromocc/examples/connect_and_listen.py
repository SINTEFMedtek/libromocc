from pyromocc import Robot, Manipulator
import numpy as np
import time

manipulator = Manipulator(Manipulator.ManipulatorType.UR10, "5.3")

robot = Robot(manipulator, "192.168.153.131", 30003)
robot.connect()

print(robot.joint_config)
time.sleep(0.5)
print(robot.joint_config)
time.sleep(0.5)

robot.movej([0.0, -np.pi/2, -np.pi/2, -np.pi/2, 0, 0], 50, 100)
