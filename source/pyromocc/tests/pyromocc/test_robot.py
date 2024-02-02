import time
import numpy as np

from unittest import TestCase

from pyromocc import Robot


class TestRobot(TestCase):
    def setUp(self) -> None:
        self.robot = Robot(ip="localhost", port=30003)
        self.robot.connect()

    def test_robot_connect(self):
        self.assertTrue(self.robot.is_connected)

    def test_operational_config(self):
        assert self.robot.operational_config is not None

    def test_robot_movej(self):
        target_q = [-np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0]
        self.robot.movej(target_q, np.pi/4, np.pi/4, wait=True)
        print(self.robot.joint_config)
        self.assertTrue(np.allclose(self.robot.joint_config, target_q, atol=0.1))

    def test_robot_movep(self):
        target_pose_aa = [-100, -450, 280, 0, -np.pi/2, 0]
        self.robot.movep(target_pose_aa, 100, 100, wait=True)
        print(self.robot.operational_config)
        self.assertTrue(np.allclose(self.robot.operational_config, target_pose_aa, atol=0.1))

    def test_robot_speedl(self):
        target_speed = [10, 0, 0, 0, 0, 0]
        self.robot.speedl(target_speed, 100, 3)

    def test_robot_speedj(self):
        target_speed_joints = [-np.pi/2, 0, 0, 0, 0, 0]
        self.robot.speedj(target_speed_joints, np.pi, time=1)

    def test_robot_speedl_stopl(self):
        target_speed = [0, 0, 50, 0, 0, 0]
        self.robot.speedl(target_speed, 500, 5)
        t0 = time.time()
        while time.time()-t0 < 2:
            print(self.robot.operational_velocity)
        self.robot.stopl()

    def test_robot_speedj_stopj(self):
        target_speed_joints = [np.pi/4, 0, 0, 0, 0, 0]
        self.robot.speedj(target_speed_joints, np.pi, time=1)
        time.sleep(2)
        self.robot.stopj()
