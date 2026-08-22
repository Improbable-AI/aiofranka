import unittest
from pathlib import Path

import mujoco
import numpy as np

from aiofranka.robot import RobotInterface


MODEL_PATH = Path(__file__).parents[1] / "aiofranka" / "model" / "fr3.xml"


class MassMatrixTest(unittest.TestCase):
    def test_mass_matrix_with_current_mujoco_api(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        data = mujoco.MjData(model)
        data.qpos[:] = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]
        mujoco.mj_forward(model, data)

        robot = RobotInterface.__new__(RobotInterface)
        robot.model = model
        robot.data = data

        mass_matrix = robot._mass_matrix()

        self.assertEqual(mass_matrix.shape, (model.nv, model.nv))
        self.assertTrue(np.isfinite(mass_matrix).all())
        np.testing.assert_allclose(mass_matrix, mass_matrix.T, rtol=0, atol=1e-12)
        np.linalg.cholesky(mass_matrix)


if __name__ == "__main__":
    unittest.main()
