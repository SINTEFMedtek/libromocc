from .pyromocc import *


class Robot(RobotBase):
    """ Robot class

    Parameters
    ----------
    ip: str
        The robot IP address.
    port: int
        Port used for communicating.
    manipulator: str, {'UR5', 'UR10'}
        Manipulator type. Currently supports UR5 and UR10 from universal robotics. Defaults to 'UR5'.
    sw_version: str
        Controller software version. Format 'major.minor'. Defaults to "5.3".
    """

    def __init__(self, ip: str, port: int = 30003, manipulator: str = None, **kwargs):
        RobotBase.__init__(self)
        self.ip = ip
        self.port = port
        self.sw_version = kwargs.get("sw_version", "5.3")

        if manipulator is None or manipulator == 'UR5':
            self.manipulator = Manipulator(ManipulatorType.UR5, self.sw_version)
        elif manipulator == 'UR10':
            self.manipulator = Manipulator(ManipulatorType.UR10, self.sw_version)
        else:
            raise ValueError("Manipulator of type {} not supported.".format(manipulator))

        self.configure(self.manipulator, self.ip, self.port)

    @property
    def joint_config(self):
        return self.get_state().get_joint_config()

    @property
    def pose(self):
        return self.get_state().get_pose()
