from unittest import TestCase
import time

from pyromocc.robot import Robot
from pyromocc.utilities.dummy_robot import DummyRobot


class TestRobot(TestCase):
    def setUp(self) -> None:
        self.fake_robot = DummyRobot()

    def test_init_robot(self):
        self.fake_robot.run()
        robot = Robot(ip="localhost", port=30003)
        robot.connect()
        time.sleep(3)
        self.fake_robot.close()
