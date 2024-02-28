import time
import warnings

try:
    import pyspacemouse
except ImportError:
    warnings.warn("pyspacemouse not installed. Please install it to use the SpaceMouse object.")


class SpaceMouse(object):
    def __init__(self):
        self.calibration_state = None
        self.is_connected = False

    def connect(self):
        self.is_connected = pyspacemouse.open()

        # Warmup
        t0 = time.time()
        while time.time()-t0 < 1.0:
            self.calibration_state = pyspacemouse.read()

    @staticmethod
    def get_state(self):
        return pyspacemouse.read()

    def get_operational_config(self, calibrated=False):
        state = self.get_state()

        if calibrated:
            return [state.x-self.calibration_state.x, state.y-self.calibration_state.y,
                    state.z-self.calibration_state.z, state.roll-self.calibration_state.roll,
                    state.pitch-self.calibration_state.pitch, state.yaw-self.calibration_state.yaw]
        else:
            return [state.x, state.y, state.z, state.roll, state.pitch, state.yaw]
