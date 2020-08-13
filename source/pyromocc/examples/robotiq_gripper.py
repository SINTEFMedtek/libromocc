from pyromocc import Robot
from pyromocc.tools import RobotiqGripper

robot = Robot(ip="10.218.93.156", port=30003, manipulator="UR5", sw_version="3.0")
robot.connect()

robotiqgrip = RobotiqGripper(robot=robot, force=255, speed=50)
robotiqgrip.open_gripper()
robotiqgrip.close_gripper()
