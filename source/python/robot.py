from .pyromocc import *


class Robot(RobotBase):
    def __init__(self, manipulator, ip, port, *args, **kwargs):
        RobotBase.__init__(self)
        self.ip = ip
        self.port = port
        self.manipulator = manipulator
        self.configure(manipulator, ip, port)

    @property
    def joint_config(self):
        return self.get_state().get_joint_config()

    @property
    def pose(self):
        return self.get_state().get_pose()
