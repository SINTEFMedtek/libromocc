import time
import warnings

from .pyromocc import *
from typing import Union

import numpy as np


class Robot(RobotBase):
    """ Robot class

    Parameters
    ----------
    ip: str
        The robot IP address.
    port: int
        Port used for communicating.
    manipulator: str, {'UR3', 'UR5', 'UR10', 'UR3e', 'UR5e', 'UR10e'}
        Manipulator type. Currently, supports UR3, UR5, and UR10 from universal robotics. Defaults to 'UR5'.
    sw_version: str
        Controller software version. Format 'major.minor'. Defaults to "5.3".

    Attributes
    ----------
    pose : ndarray
        Returns the instant 4x4 homogeneous matrix from base to end-effector. The pose is in mm.
    pose_aa : list
        A list of 6 values representing the axis-angle representation and translation of the flange wrt base. The pose
        is in mm.
    joint_config : ndarray
        Returns the instant joint configuration of the robot.
    joint_velocity : ndarray
        Returns the instant joint velocity of the robot.
    operational_config : ndarray
        Returns the instant operational configuration of the robot.
    operational_velocity : ndarray
        Returns the instant operational velocity of the robot.
    operational_force : ndarray
        Returns the instant operational force of the robot.


    Methods
    -------
    connect()
        Triggers robot connection process. Checks and confirms connection validity within a 5-second loop.
        Raises a warning if the robot is not connected after 5 seconds. Returns connection status.
    move_to_pose(pose, acceleration, velocity, wait=False, pose_units="mm")
        Move to the specified pose. Pose is a 4x4 homogeneous transform.
    translate(vec, acceleration, velocity, wait=False)
        Translate the end-effector by the specified vector (x, y, z).
    movep(pose, acceleration, velocity, time=0, blend_radius=0, wait=False)
        Move the robot to the specified pose.

    """

    def __init__(self, ip: str, port: int = 30003, manipulator: str = None, sw_version="3.15"):
        RobotBase.__init__(self)
        self.ip = ip
        self.port = port
        self.sw_version = sw_version
        self.is_connected = False

        if manipulator is None:
            manipulator = "UR5"

        manipulator_type = self._string_to_manipulator_type(manipulator)

        self.manipulator = Manipulator(manipulator_type, self.sw_version)
        self.configure(self.manipulator, self.ip, self.port)

    def connect(self):
        """ Triggers robot connection process. Checks and confirms connection validity within a 5-second loop.
        Raises a warning if the robot is not connected after 5 seconds. Returns connection status.
        """

        self.is_connected = self._connect()

        t0 = time.time()
        while not self._has_valid_state() or time.time()-t0 > 5.0:
            time.sleep(0.1)
            self.is_connected = self._has_valid_state()

        if not self.is_connected:
            warnings.warn("Robot not connected. Please check the IP and port.")

        return self.is_connected

    def move_to_pose(self, pose, acceleration, velocity, wait=False, pose_units="mm"):
        """
        Parameters
        ----------
        pose: ndarray
            Move to the specified pose. Pose is a 4x4 homogeneous transform
        acceleration: float
            Acceleration to velocity value
        velocity: float
            Velocity of motion
        wait: bool
            Wait for the motion to finish before returning
        pose_units: str
            Units of the pose, 'mm' or 'm'. Default is 'mm'.
        """

        if pose_units == "mm":
            self.movep(pose, acceleration, velocity, 0, 0, wait)
        elif pose_units == "m":
            # scale to mm before calling movep
            pose[:3, 3] *= 1000
            acceleration *= 1000
            velocity *= 1000
            self.movep(pose, acceleration, velocity, 0, 0, wait)
        else:
            raise NotImplemented("Unit {} not supported.".format(pose_units))

    def translate(self, vec, acceleration, velocity, wait=False):
        """
        Translate the end-effector by the specified vector (x, y, z).

        Parameters
        ----------
        vec: np.ndarray, sequence (list, tuple)
            Translate with the relative vector (x, y, z)
        acceleration: float
            Acceleration to velocity value (mm/s^2)
        velocity: float
            Velocity of motion (mm/s)
        wait: bool
            Wait for the motion to finish before returning
        """
        pose = self.pose
        pose[:3, 3] += vec
        self.movep(pose, acceleration, velocity, 0, 0, wait)

    def movep(self, pose, acceleration, velocity, time=0, blend_radius=0, wait=False):
        """
        Move the robot to the specified pose.

        Parameters
        ----------
        pose: np.ndarray, Sequence
            Move to the specified pose. Pose is a 4x4 homogeneous transform or a Sequence of 6 values representing the
            axis-angle representation and translation of the flange wrt base.
        acceleration: float
            Acceleration to velocity value
        velocity: float
            Velocity of motion (unit: mm/s)
        time: float
            Time to complete the motion. If 0, the robot will move with the specified velocity and acceleration.
        blend_radius: float
            Blend radius for the motion.
        wait: bool
            Wait for the motion to finish before returning.
        """
        self._movep(pose, acceleration, velocity, time, blend_radius, wait)

    def movej(self, joint_config, acceleration, velocity, time=0, blend_radius=0, wait=False):
        """
        Move the robot to the specified joint configuration.

        Parameters
        ----------
        joint_config: np.ndarray, Sequence
            Move to the specified joint configuration.
        acceleration: float
            Acceleration to velocity value (rad/s^2)
        velocity: float
            Velocity of motion (rad/s)
        time: float
            Time to complete the motion. If 0, the robot will move with the specified velocity and acceleration.
        blend_radius: float
            Blend radius for the motion.
        wait: bool
            Wait for the motion to finish before returning.
        """
        self._movej(joint_config, acceleration, velocity, time, blend_radius, wait)

    def speedl(self, velocity, acceleration, time=0.5):
        """
        Set the linear speed of the robot.

        Parameters
        ----------
        velocity: np.ndarray, Sequence
            Linear velocity (x, y, z, rx, ry, rz) [mm/s, rad/s]
        acceleration: float
            Linear acceleration (mm/s^2)
        time: float
            Time to keep motion regardless of the velocity is reached.
        """
        self._speedl(velocity, acceleration, time)

    def speedj(self, velocity, acceleration, time=0.5):
        """
        Set the joint speed of the robot.

        Parameters
        ----------
        velocity: np.ndarray, Sequence
            Joint velocity (rad/s)
        acceleration: float
            Joint acceleration (rad/s^2)
        time: float
            Time to keep motion regardless of the velocity is reached.
        """
        self._speedj(velocity, acceleration, time)

    def stopl(self, acceleration=500):
        """
        Stops the robot with linear motion and given acceleration.

        Parameters
        ----------
        acceleration: float
            Linear acceleration (mm/s^2)
        """
        self._stopl(acceleration)

    def stopj(self, acceleration=np.pi/4):
        """
        Stops the robot with joint motion and given joint acceleration.

        Parameters
        ----------
        acceleration: float
            Joint acceleration (rad/s^2)
        """
        self._stopj(acceleration)

    @property
    def joint_config(self):
        return self.get_state().get_joint_config()

    @property
    def joint_velocity(self):
        return self.get_state().get_joint_velocity()

    @property
    def operational_config(self):
        return self.get_state().get_operational_config()

    @property
    def operational_velocity(self):
        return self.get_state().get_operational_config()

    @property
    def operational_force(self):
        return self.get_state().get_operational_force()

    @property
    def pose(self):
        pose = np.copy(self.get_state().get_pose())
        return pose

    def get_pose(self, pose_units="mm"):
        pose = np.copy(self.get_state().get_pose())
        if pose_units == "mm":
            return pose
        elif pose_units == "m":
            pose[:3, 3] /= 1000
            return pose
        else:
            raise NotImplemented("Unit {} not supported.".format(pose_units))

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
        self.movep(p, 50, 100, 0, 0, True)

    @property
    def y(self):
        return self.pose[1, 3]

    @y.setter
    def y(self, val):
        p = self.pose
        p[1, 3] = val
        self.movep(p, 50, 100, 0, 0, True)

    @property
    def z(self):
        return self.pose[2, 3]

    @z.setter
    def z(self, val):
        p = self.pose
        p[2, 3] = val
        self.movep(p, 50, 100, 0, 0, True)

    @property
    def rx(self):
        return self.pose_aa[3]

    @rx.setter
    def rx(self, val):
        p = self.pose_aa
        p[3] = val
        self.movep(p, 50, 100, 0, 0, True)

    @property
    def ry(self):
        return self.pose_aa[4]

    @ry.setter
    def ry(self, val):
        p = self.pose_aa
        p[4] = val
        self.movep(p, 50, 100, 0, 0, True)

    @property
    def rz(self):
        return self.pose_aa[5]

    @rz.setter
    def rz(self, val):
        p = self.pose_aa
        p[5] = val
        self.movep(p, 50, 100, 0, 0, True)

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
        """ Returns the Jacobian matrix of the robot."""
        return self.get_state().get_jacobian()

    def inverse_jacobian(self):
        """Returns the inverse Jacobian matrix of the robot."""
        return self.get_state().get_inverse_jacobian()

    def send_program(self, program: Union[str, bytes]):
        """ Sends program to the robot in URScript format. """
        program.strip()
        if not isinstance(program, bytes):
            program = program.encode()
        self._send_program(program)

    def _has_valid_state(self):
        """ Checks if robot is in a valid state by evaluating the joint configuration."""
        joint_config = self.joint_config
        return not np.any(np.abs(joint_config) > 2*np.pi)

    @staticmethod
    def _string_to_manipulator_type(manipulator):
        """ Converts user provided manipulator type string to enumerated type. """

        if manipulator == 'UR3':
            return ManipulatorType.UR3
        elif manipulator == 'UR3e':
            return ManipulatorType.UR3e
        elif manipulator == 'UR5':
            return ManipulatorType.UR5
        elif manipulator == 'UR5e':
            return ManipulatorType.UR5e
        elif manipulator == 'UR10':
            return ManipulatorType.UR10
        elif manipulator == 'UR10e':
            return ManipulatorType.UR10e
