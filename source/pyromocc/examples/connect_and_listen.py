import time
import numpy as np
import argparse

from pyromocc import Robot

np.set_printoptions(precision=3)

parser = argparse.ArgumentParser(description='UR robot listener example')
parser.add_argument('--ip', type=str, default="192.168.199.129", help='Robot IP address')
parser.add_argument('--port', type=int, default=30003, help='Robot port number')
parser.add_argument('--manipulator', type=str, default="UR5", help='Manipulator type')
parser.add_argument('--sw_version', type=str, default="3.15", help='Controller software version')

args = parser.parse_args()

robot = Robot(ip=args.ip, port=args.port, manipulator=args.manipulator, sw_version=args.sw_version)
robot.connect()

t0 = time.time()
while time.time()-t0 < 5:
    print(f"Joint config: {robot.joint_config}")
    print(f"Joint velocity: {robot.joint_velocity}")
    print(f"Operational config {robot.operational_config}")
    print(f"Operational velocity {robot.operational_velocity}")
    print(f"Operational force {robot.operational_force} \n")
