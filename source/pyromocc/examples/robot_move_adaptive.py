from pyromocc import Robot
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

np.set_printoptions(precision=3)
fig = plt.figure()
ax3d = fig.add_subplot(1, 2, 1, projection='3d')

figj, axj = plt.subplots(nrows=2, ncols=6)

robot = Robot(ip="192.168.153.128", port=30003, manipulator="UR5", sw_version="3.0")
robot.connect()

sleep(2.0)
print(robot.joint_config)

target_q = [0.0, -np.pi/2, -np.pi/2, -np.pi/2, 0, 0]
target_pose_aa = robot.forward_kinematics(target_q, format='axis_angle')

robot.movej(target_q, 5, 10)
print(robot.inverse_jacobian())
print(robot.jacobian())

count = 0
while count < 1000:
    x, y, z, rx, ry, rz = robot.pose_aa
    ax3d.scatter(x, y, z, color='b')
    ax3d.scatter(target_pose_aa[0], target_pose_aa[1], target_pose_aa[2], color='r')

    axj[0, 0].plot(count, robot.joint_config[0], 'ro')
    axj[0, 1].plot(count, robot.joint_config[1], 'ro')
    axj[0, 2].plot(count, robot.joint_config[2], 'ro')
    axj[0, 3].plot(count, robot.joint_config[3], 'ro')
    axj[0, 4].plot(count, robot.joint_config[4], 'ro')
    axj[0, 5].plot(count, robot.joint_config[5], 'ro')

    axj[1, 0].plot(count, robot.joint_velocity[0], 'ro')
    axj[1, 1].plot(count, robot.joint_velocity[1], 'ro')
    axj[1, 2].plot(count, robot.joint_velocity[2], 'ro')
    axj[1, 3].plot(count, robot.joint_velocity[3], 'ro')
    axj[1, 4].plot(count, robot.joint_velocity[4], 'ro')
    axj[1, 5].plot(count, robot.joint_velocity[5], 'ro')

    count += 1
    plt.draw()
    plt.pause(1/125.0)

sleep(1.0)
print(robot.joint_config)
print(robot.pose)
