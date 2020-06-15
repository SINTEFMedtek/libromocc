from unittest import TestCase
import numpy as np

from pyromocc import CalibrationMethods


class TestCalibration(TestCase):
    def setUp(self) -> None:
        pass

    def test_calibration_shah(self):
        poses_a = np.random.random((100, 4, 4))
        poses_b = poses_a

        calib_matrices = CalibrationMethods.calibration_shah(poses_a, poses_b)
        assert np.allclose(calib_matrices.pose_x, np.eye(4, 4))
        assert np.allclose(calib_matrices.pose_y, np.eye(4, 4))
        calib_errors = CalibrationMethods.estimate_calibration_error(calib_matrices.pose_x, calib_matrices.pose_y,
                                                                     poses_a, poses_b)
        print(calib_errors.translation_error, calib_errors.rotation_error)
