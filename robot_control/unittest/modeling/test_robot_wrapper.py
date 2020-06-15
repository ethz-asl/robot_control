import unittest
import numpy as np
from rc import RobotWrapper
from robot_control_assets import RobotControlAssets as rca


PANDA_URDF_PATH = rca.get_panda_urdf_path()


class RobotWrapperTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wrapped = RobotWrapper(PANDA_URDF_PATH)

    def test_loads(self):
        self.assertEqual(self.wrapped.get_dof(), 9)
        neutral = self.wrapped.get_neutral_configuration()
        np.testing.assert_almost_equal(neutral, np.zeros(9))
        self.assertEqual(neutral.shape, self.wrapped.get_random_configuration().shape)

    def test_jacobian(self):
        J = self.wrapped.get_frame_jacobian('panda_link8')
        self.assertEqual(J.shape[0], 6)
        self.assertEqual(J.shape[1], 9)
        self.assertGreater(np.abs(J.sum()), 0.5)
        self.wrapped.update_state(np.ones(9) * 0.25, np.zeros(9))
        J2 = self.wrapped.get_frame_jacobian('panda_link8')
        self.assertGreater(np.abs(J - J2).sum(), 0.5)
        np.testing.assert_allclose(np.ones(9) * 0.25, self.wrapped.get_q())
        np.testing.assert_allclose(np.zeros(9), self.wrapped.get_v())


if __name__ == '__main__':
    import rostest
    rostest.rosrun('robot_control', 'robot_wrapper_test', RobotWrapperTestCase)
