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

    @property
    def pose_aa(self):
        pose = self.get_state().get_pose()
        return pose_to_vector(pose)

    @property
    def x(self):
        return self.pose[0, 3]

    @x.setter
    def x(self, val):
        p = self.pose
        p[0, 3] = val
        self.movep(p, 50, 100)

    @property
    def y(self):
        return self.pose[1, 3]

    @y.setter
    def y(self, val):
        p = self.pose
        p[1, 3] = val
        self.movep(p, 50, 100)

    @property
    def z(self):
        return self.pose[2, 3]

    @z.setter
    def z(self, val):
        p = self.pose
        p[2, 3] = val
        self.movep(p, 50, 100)

    @property
    def rx(self):
        return self.pose_aa[3]

    @rx.setter
    def rx(self, val):
        p = self.pose_aa
        p[3] = val
        self.movep(p, 50, 100)

    @property
    def ry(self):
        return self.pose_aa[4]

    @ry.setter
    def ry(self, val):
        p = self.pose_aa
        p[4] = val
        self.movep(p, 50, 100)

    @property
    def rz(self):
        return self.pose_aa[5]

    @rz.setter
    def rz(self, val):
        p = self.pose_aa
        p[5] = val
        self.movep(p, 50, 100)