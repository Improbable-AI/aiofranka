import mujoco 
import mujoco.viewer
from pathlib import Path
import numpy as np 
import time 

CUR_DIR = Path(__file__).parent.resolve()

class RobotInterface: 

    def __init__(self, ip = None): 

        self.real = ip is not None

        self.model = mujoco.MjModel.from_xml_path(f"{CUR_DIR}/model/fr3.xml")
        self.data = mujoco.MjData(self.model)

        self.torque_controller = None

        # End-effector site we wish to control.
        self.site_name = "attachment_site"
        self.site_id = self.model.site(self.site_name).id

        if self.real: 
            import pylibfranka
            self.robot = pylibfranka.Robot(ip, pylibfranka.RealtimeConfig.kIgnore)

            self.robot.set_collision_behavior(
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            )
            
            self.sync_mj()


        else: 
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.data.qpos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
            mujoco.mj_forward(self.model, self.data)
            self.viewer.sync() 
        
    def start(self): 
        if self.real:
            self.torque_controller = self.robot.start_torque_control()

    def stop(self): 
        if self.real:
            self.robot.stop()

    def sync_mj(self): 
        """ Sync mujoco state with real robot state """

        if self.torque_controller is None:
            robot_state = self.robot.read_once()
        else:
            robot_state, _ = self.torque_controller.readOnce()

        self.data.qpos = np.array(robot_state.q)
        self.data.qvel = np.array(robot_state.dq)
        self.data.ctrl = np.array(robot_state.tau_J_d)
        mujoco.mj_forward(self.model, self.data)

    @property 
    def state(self): 
        """ Get current robot state """

        if self.real: 
            self.sync_mj()

        state = { 
            "qpos": np.array(self.data.qpos),
            "qvel": np.array(self.data.qvel), 
            "ee": self._ee(),
            "jac": self._jacobian(),
            "mm": self._mass_matrix(), 
            "last_torque": np.array(self.data.ctrl),
        }

        return state 

    def _mass_matrix(self): 
        """ Compute mass matrix at current state """

        mm = np.zeros((7,7))
        mujoco.mj_fullM(self.model, mm, self.data.qM)
        return mm

    def _ee(self):
        ee_xyz = self.data.site(self.site_id).xpos
        ee_mat = self.data.site(self.site_id).xmat.reshape(3,3)
        ee = np.eye(4)
        ee[:3, :3] = ee_mat
        ee[:3, 3] = ee_xyz

        return ee

        
    def _jacobian(self):
        jac = np.zeros((6, 7))
        mujoco.mj_jacSite(self.model, self.data, jac[:3], jac[3:], self.site_id)
        return jac

    def step(self, torque: np.ndarray): 
        """ Step the simulation or send torque command to real robot """

        if self.real: 
            import pylibfranka
            torque_command = pylibfranka.Torques(torque.tolist())
            torque_command.motion_finished = False
            self.torque_controller.writeOnce(torque_command)
        else: 
            self.data.ctrl = torque
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()

if __name__ == "__main__": 

    robot = RobotInterface("172.16.0.2")
    while True: 

        zero_torque = np.zeros(7)
        robot.step(zero_torque)
        time.sleep(0.1)
        print(zero_torque)