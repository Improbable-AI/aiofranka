import os
import mujoco 
import mujoco.viewer
from pathlib import Path
import numpy as np 
import time
import requests 
from aiofranka.client import FrankaLockUnlock

CUR_DIR = Path(__file__).parent.resolve()


class RobotInterface: 
    """
    High-level interface for Franka FR3 robot control.
    
    This class provides a unified interface for both real robot control (via pylibfranka)
    and simulation (via MuJoCo). It handles low-level communication, state synchronization,
    and kinematics/dynamics computation.
    
    Attributes:
        real (bool): True if connected to real robot, False for simulation
        model (mujoco.MjModel): MuJoCo model for kinematics/dynamics
        data (mujoco.MjData): MuJoCo data structure with current state
        robot (pylibfranka.Robot): Real robot interface (if real=True)
        torque_controller: Active torque control interface
        site_name (str): Name of end-effector site in MuJoCo model
        site_id (int): MuJoCo site ID for end-effector
        viewer (mujoco.viewer): MuJoCo viewer window (if real=False)
        
    Examples:
        Real robot:
            >>> robot = RobotInterface("172.16.0.2")
            >>> robot.start()
            >>> state = robot.state
            >>> robot.step(np.zeros(7))  # Send zero torques
            >>> robot.stop()
            
        Simulation:
            >>> robot = RobotInterface(None)
            >>> state = robot.state
            >>> robot.step(np.zeros(7))  # Updates MuJoCo simulation
        
    Caveats:
        - Must call start() before step() on real robot
        - Collision behavior is set to high thresholds by default
        - State is synced from robot on every access (thread-safe)
        - MuJoCo model must match real robot configuration
    """

    def __init__(self, ip = None): 
        """
        Initialize robot interface.
        
        Args:
            ip (str | None): Robot IP address (e.g., "172.16.0.2") for real robot,
                           or None for simulation mode.
                           
        Raises:
            RuntimeError: If pylibfranka is not installed (real robot mode)
            ConnectionError: If cannot connect to robot at given IP
            
        Note:
            In real mode, collision thresholds are set to [100.0] * 7 for joints
            and [100.0] * 6 for Cartesian space. Adjust via robot.robot.set_collision_behavior()
            for more conservative behavior.
        """

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
        """
        Start torque control mode on the real robot.
        
        This must be called before sending torque commands. Does nothing in simulation.
        
        Raises:
            RuntimeError: If robot is not ready or already in control mode
            
        Caveat:
            After calling start(), you must send torque commands at ~1kHz to maintain
            control. Use FrankaController for automatic control loop management.
        """
        if self.real:
            self.torque_controller = self.robot.start_torque_control()

    def stop(self): 
        """
        Stop torque control mode on the real robot.
        
        This gracefully terminates the control session. Does nothing in simulation.
        
        Caveat:
            Robot will hold position briefly then release brakes. Ensure robot
            is in a safe configuration before stopping.
        """
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
        """
        Get current robot state with kinematics and dynamics.
        
        Returns:
            dict: Dictionary containing:
                - qpos (np.ndarray): Joint positions [rad] (7,)
                - qvel (np.ndarray): Joint velocities [rad/s] (7,)
                - ee (np.ndarray): End-effector pose as 4x4 homogeneous transform
                                  [[R, p], [0, 1]] where R is rotation, p is position
                - jac (np.ndarray): End-effector Jacobian (6, 7) - [linear; angular]
                - mm (np.ndarray): Joint-space mass matrix (7, 7)
                - last_torque (np.ndarray): Last commanded torques [Nm] (7,)
                
        Note:
            State is synchronized from real robot on every access. MuJoCo model
            is updated with latest robot state before computing kinematics/dynamics.
            
        Example:
            >>> state = robot.state
            >>> print(f"Joint 1 position: {state['qpos'][0]:.3f} rad")
            >>> print(f"EE position: {state['ee'][:3, 3]}")
            >>> print(f"EE orientation: {state['ee'][:3, :3]}")
        """

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
        """
        Send torque command to robot or step simulation.
        
        Args:
            torque (np.ndarray): Joint torques [Nm] (7,)
            
        Raises:
            RuntimeError: If real robot not started or communication error
            
        Note:
            - Real robot: Sends torque command via pylibfranka at current timestep
            - Simulation: Updates MuJoCo with torques and advances one timestep
            
        Caveats:
            - Must be called at ~1kHz for real robot to maintain control
            - Large torque changes may trigger safety limits
            - Torques should respect robot limits: |tau_i| < 87 Nm for joints 1-4,
              |tau_i| < 12 Nm for joints 5-7
              
        Example:
            >>> # Send gravity compensation torques
            >>> torque = robot.state['mm'] @ np.array([0, 0, 0, 0, 0, 0, -9.81])
            >>> robot.step(torque)
        """

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