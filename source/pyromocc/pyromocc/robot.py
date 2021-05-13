from .pyromocc import *
from typing import Union


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

    Attributes
    ----------
    pose : ndarray
        Returns the instant 4x4 homogeneous matrix from base to end-effector
    pose_aa : list
        A list of 6 values representing the axis-angle representation and translation of the end-effector wrt base
    """

    def __init__(self, ip: str, port: int = 30003, manipulator: str = None, units="mm", **kwargs):
        RobotBase.__init__(self)
        self.ip = ip
        self.port = port
        self.sw_version = kwargs.get("sw_version", "5.3")
        self.units = units  # default unit is mm (millimetre)

        if manipulator is None or manipulator == 'UR5':
            self.manipulator = Manipulator(ManipulatorType.UR5, self.sw_version)
        elif manipulator == 'UR10':
            self.manipulator = Manipulator(ManipulatorType.UR10, self.sw_version)
        else:
            raise ValueError("Manipulator of type {} not supported.".format(manipulator))

        self.configure(self.manipulator, self.ip, self.port)

    def move_to_pose(self, pose, acceleration, velocity):
        """
        Parameters
        ----------
        pose: ndarray
            Move to the specified pose. Pose is a 4x4 homogeneous transform
        acceleration: float
            Acceleration to velocity value
        velocity: float
            Velocity of motion
        """

        if self.units == "mm":
            self.movep(pose, acceleration, velocity)
        elif self.units == "m":
            # scale to mm before calling movep
            pose[:3, 3] *= 1000
            acceleration *= 1000
            velocity *= 1000
            self.movep(pose, acceleration, velocity)
        else:
            raise NotImplemented("Unit {} not supported.".format(self.units))

    @property
    def joint_config(self):
        return self.get_state().get_joint_config()

    @property
    def joint_velocity(self):
        return self.get_state().get_joint_velocity()

    @property
    def pose(self):
        pose = self.get_state().get_pose()
        if self.units == "mm":
            return pose
        elif self.units == "m":
            pose[:3, 3] /= 1000
            return pose
        else:
            raise NotImplemented("Unit {} not supported.".format(self.units))

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

    def forward_kinematics(self, joint_config, format="homogeneous"):
        pose = self.get_state().joint_to_pose(joint_config)
        if format == "axis_angle":
            return pose_to_vector(pose)
        return pose

    def inverse_kinematics(self, pose):
        pose[:, 3][:3] = pose[:, 3][:3] / 1000
        joint_config = self.get_state().pose_to_joint(pose)
        return joint_config

    def jacobian(self):
        return self.get_state().get_jacobian()

    def inverse_jacobian(self):
        return self.get_state().get_inverse_jacobian()

    def send_program(self, program: Union[str, bytes]):
        """
        Sends program to the robot in URScript format.
        """
        program.strip()
        if not isinstance(program, bytes):
            program = program.encode()
        self._send_program(program)
