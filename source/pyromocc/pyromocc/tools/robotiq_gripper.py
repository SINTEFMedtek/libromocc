import time
from .urscript import URScript


class RobotiqScript(URScript):
    """
    Object for generating scripts for the Robotiq gripper. Adapted from python-urx.
    """

    def __init__(self, socket_host="127.0.0.1", socket_port=63352, socket_name="gripper_socket"):
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        super(RobotiqScript, self).__init__()

        # Reset connection to gripper
        self.socket_close(self.socket_name)
        self.socket_open(self.socket_host, self.socket_port, self.socket_name)

    def set_gripper_activate(self):
        self.socket_set_var("GTO", 1, self.socket_name)

    def set_gripper_force(self, value):
        """
        FOR is the variable
        range is 0 - 255
        0 is no force
        255 is full force
        """
        value = self._constrain_unsigned_char(value)
        self.socket_set_var("FOR", value, self.socket_name)

    def set_gripper_position(self, value):
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        value = self._constrain_unsigned_char(value)
        self.socket_set_var("POS", value, self.socket_name)

    def set_gripper_speed(self, value):
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        value = self._constrain_unsigned_char(value)
        self.socket_set_var("SPE", value, self.socket_name)

    def set_robot_activate(self):
        self.socket_set_var("ACT", 1, self.socket_name)


class RobotiqGripper(object):
    def __init__(self, robot, payload=0.85, speed=255, force=50, socket_host="127.0.0.1", socket_port=63352,
                 socket_name="gripper_socket"):

        self.robot = robot
        self.payload = payload
        self.speed = speed
        self.force = force
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name

    def _setup_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript(socket_host=self.socket_host, socket_port=self.socket_port,
                                 socket_name=self.socket_name)

        # Set input and output voltage ranges
        urscript.set_analog_input_range(0, 0)
        urscript.set_analog_input_range(1, 0)
        urscript.set_analog_input_range(2, 0)
        urscript.set_analog_input_range(3, 0)
        urscript.set_analog_outputdomain(0, 0)
        urscript.set_analog_outputdomain(1, 0)
        urscript.set_tool_voltage(0)
        urscript.set_runstate_outputs()

        # Set payload, speed and force
        urscript.set_payload(self.payload)
        urscript.set_gripper_speed(self.speed)
        urscript.set_gripper_force(self.force)

        # Initialize the gripper
        urscript.set_robot_activate()
        urscript.set_gripper_activate()

        # Wait on activation to avoid USB conflicts
        urscript.sleep(0.1)

        return urscript

    def gripper_action(self, value):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        urscript = self._setup_urscript()

        # Move to the position
        sleep = 2.0
        urscript.set_gripper_position(value)
        urscript.sleep(sleep)

        # Send the script
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(sleep)

    def open_gripper(self):
        self.gripper_action(0)

    def close_gripper(self):
        self.gripper_action(255)
