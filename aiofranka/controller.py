import numpy as np
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
import sys 
sys.path.append(".")
import threading
import asyncio
import time
from tqdm import trange
from pathlib import Path
import numpy as np 
import time 
from aiofranka.robot import RobotInterface
from ruckig import InputParameter, Ruckig, Trajectory, Result

CUR_DIR = Path(__file__).parent.resolve()



class FrankaController: 

    def __init__(self, robot: RobotInterface):
        """
        Initialize Impedance Controller with pre-initialized resources.
        
        Args:
            robot: pylibfranka Robot instance
            model: pylibfranka Model for dynamics computation
        """
        self.robot = robot

        self.initialize() 
        self.state_lock = threading.Lock()

        self.type = "impedance"
        self.running = False
        self.task = None
        self.clip = True 


        self.kp, self.kd = np.ones(7) * 80, np.ones(7) * 4
        self.ee_kp, self.ee_kd = np.ones(6) * 100, np.ones(6) * 4
        self.null_kp, self.null_kd = np.ones(7) * 1, np.ones(7) * 1


        self.track = False 
        self.torque_diff_limit = 990.
        
        # Rate limiting for .set() method
        self._update_freq = 50.0  # Default 50Hz
        self._last_update_time = {}
        self._pending_updates = {}

        self.state = None 


    async def test_connection(self): 

        self.track = True 
        await asyncio.sleep(5)
        self.track = False

    def initialize(self): 

        # Get initial state
        initial_state = self.robot.state
        self.initial_ee = initial_state['ee']
        self.initial_qpos = deepcopy(initial_state['qpos'])
        self.initial_qvel = deepcopy(initial_state['qvel'])

        self.q_desired= self.initial_qpos
        self.ee_desired = self.initial_ee

    def _update_desired(self, desired):
        """
        Update the desired joint positions, kp and kd
        This function is called by the server when a client sends new values
        Thread-safe update of shared state.
        
        Args:
            desired: Desired joint positions (7-element array)
            kp: Joint position stiffness gains (7-element array)
            kd: Joint velocity damping gains (7-element array)
        """
        with self.state_lock:
            self.q_desired = np.array(desired) if type(desired) == list else desired
    
    def set_freq(self, freq: float):
        """Set the update frequency for rate-limited setters.
        
        Args:
            freq: Desired update frequency in Hz
        """
        self._update_freq = freq
    
    async def set(self, attr: str, value):
        """Rate-limited setter that enforces strict timing.
        
        This method ensures updates to controller attributes happen at the
        frequency specified by set_freq(). It compensates for drift by tracking
        the target time for each update.
        
        Args:
            attr: Attribute name (e.g., 'q_desired', 'ee_desired')
            value: Value to set
        """
        current_time = time.perf_counter()
        dt = 1.0 / self._update_freq
        
        # Initialize tracking for this attribute if first time
        if attr not in self._last_update_time:
            self._last_update_time[attr] = current_time
            # Sleep for the first update too to maintain consistent timing
            await asyncio.sleep(dt)
            with self.state_lock:
                setattr(self, attr, value)
            self._last_update_time[attr] = current_time + dt
            return
        
        # Calculate target time for this update
        target_time = self._last_update_time[attr] + dt
        
        # Sleep until target time
        sleep_time = target_time - current_time
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        
        setattr(self, attr, value)
        
        # Update last update time to target (not actual) to avoid drift
        self._last_update_time[attr] = target_time


    async def _run(self): 
        """Run the control loop continuously in the background"""
        self.running = True
        
        loop_times = []
        last_time = time.perf_counter()
        log_interval = 1000  # Log every 1000 iterations (1 second at 1000Hz)
        iteration = 0
        
        try:
            while self.running: 

                
                t0 = time.time() 
                self.step()
                
                # Track timing
                if self.track:
                    current_time = time.perf_counter()
                    dt = current_time - last_time
                    loop_times.append(dt)
                    last_time = current_time
                    iteration += 1
                    
                    # Log statistics every log_interval iterations
                    if iteration % log_interval == 0:
                        loop_times_array = np.array(loop_times)
                        mean_dt = np.mean(loop_times_array) * 1000  # Convert to ms
                        std_dt = np.std(loop_times_array) * 1000
                        min_dt = np.min(loop_times_array) * 1000
                        max_dt = np.max(loop_times_array) * 1000
                        actual_freq = 1.0 / np.mean(loop_times_array)
                        
                        print(f"Control loop stats (last {log_interval} iterations):")
                        print(f"  Frequency: {actual_freq:.1f} Hz (target: 1000 Hz)")
                        print(f"  Mean dt: {mean_dt:.3f} ms, Std: {std_dt:.3f} ms")
                        print(f"  Min dt: {min_dt:.3f} ms, Max dt: {max_dt:.3f} ms")
                        print(f"  Jitter (max-min): {max_dt - min_dt:.3f} ms")
                        
                        loop_times.clear()

                dt = time.time() - t0
                if not self.robot.real: 
                    await asyncio.sleep(1/1000. - dt)  # Yield control to event loop
                else:
                    await asyncio.sleep(0)  # Yield control to event loop
        except Exception as e:
            self.running = False
            print(f"Error in control loop: {e}")
            # diff = (self.torque - self.last_torque)/1e-3
            # diff = np.abs(diff)

            # if np.any(diff > 1000.):
            #     # print what axis is causing the issue
            #     arg_idxs = np.where(diff > 1000.)[0]
            #     print(f"High torque rate of change detected on axes: {arg_idxs}")
            #     print((self.torque - self.last_torque)/1e-3) 
            sys.exit(1)  # Kill the entire script
    
    async def start(self):
        """Start the background control loop"""

        print("starting robot!")
        self.robot.start()

        if self.task is None or self.task.done():
            self.task = asyncio.create_task(self._run())
        await asyncio.sleep(1)  # Yield to ensure the task starts
        return self.task
    
    async def stop(self):
        """Stop the background control loop"""
        self.running = False
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                print("Control loop task cancelled.") 

        self.robot.stop()
        print("robot stopped")
        await asyncio.sleep(1)  # Yield to ensure the task starts

    def switch(self, controller_type: str):
        self.type = controller_type
        self.initialize()
        # Reset timing state when switching controllers
        self._last_update_time.clear()

    def step(self): 
        self.state = self.robot.state
        if self.type == "impedance":
            self._impedance_step(self.state)
        elif self.type == "osc":
            self._osc_step(self.state)
        elif self.type == "torque":
            self._torque_step(self.state)
        else:
            raise ValueError(f"Unknown controller type: {self.type}")

    def _torque_step(self, state):

        self.robot.step(self.torque)

    def _osc_step(self, state): 

        jac = state['jac']
        ee = state['ee']
        q = state['qpos']
        dq = state['qvel']
        mm = state['mm']
        last_torque = state['last_torque']


        with self.state_lock:
            ee_goal = self.ee_desired

        position_error = ee_goal[:3, 3] - ee[:3, 3]

        rotation_error = R.from_matrix(ee_goal[:3, :3]) * R.from_matrix(ee[:3, :3]).inv()
        rotation_error_vec = rotation_error.as_rotvec()

        twist = np.zeros(6)
        twist[:3] = position_error
        twist[3:] = rotation_error_vec
        ee_vel = jac @ dq
        # minv = np.linalg.inv(mm)
        if abs(np.linalg.det(mm)) > 1e-2:
            minv = np.linalg.inv(mm)
        else:
            minv = np.linalg.pinv(mm)
        mx_inv = jac @ minv @ jac.T

        if abs(np.linalg.det(mx_inv)) > 1e-2:
            mx = np.linalg.inv(mx_inv)
        else:
            mx = np.linalg.pinv(mx_inv)

        # operational space feedback torque
        feedback = jac.T @ mx @ (self.ee_kp * twist - self.ee_kd * ee_vel)

        # null space torque
        q0 = self.initial_qpos
        ddq = self.null_kp * (q0 - q) - self.null_kd * dq
        jbar = minv @ jac.T @ mx
        null = (np.eye(7) - jac.T @ jbar.T) @ ddq

        # Add coriolis compensation
        tau_d = feedback + null

        # make sure torque rate of change is not too high
        if self.clip:
            diff = (tau_d - last_torque)/1e-3
            diff = np.clip(diff, -self.torque_diff_limit, self.torque_diff_limit)
            tau_d = last_torque + diff * 1e-3

        self.robot.step(tau_d)

    


    def _impedance_step(self, robot_state): 


        # Get state variables
        q = np.array(robot_state['qpos'])
        dq = np.array(robot_state['qvel'])
        last_torque = robot_state['last_torque']

        # Get current target from trajectory (thread-safe)
        with self.state_lock:
            kp = self.kp
            kd = self.kd
            q_desired = self.q_desired
            q_goal = q_desired
    
        position_error = q_goal - q

        # Compute joint-space impedance control
        tau = position_error * kp - dq * kd

        # Add coriolis compensation
        tau_d = tau #+ coriolis
        
        # make sure torque rate of change is not too high
        if self.clip:
            diff = (tau_d - last_torque)/1e-3
            diff = np.clip(diff, -self.torque_diff_limit, self.torque_diff_limit)
            tau_d = last_torque + diff * 1e-3

        self.torque = tau_d

        self.robot.step(tau_d)


    async def move(self, qpos = [0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853]):
        self.type = "impedance"

        inp = InputParameter(7)

        inp.current_position = self.robot.state['qpos']
        inp.current_velocity = self.robot.state['qvel']
        inp.current_acceleration = np.zeros(7)

        inp.target_position = np.array(qpos)
        inp.target_velocity = np.zeros(7)
        inp.target_acceleration = np.zeros(7)

        inp.max_velocity = np.ones(7) * 10
        inp.max_acceleration = np.ones(7) * 5 
        inp.max_jerk = np.ones(7)
        
        otg = Ruckig(7)
        trajectory = Trajectory(7)

        result = otg.calculate(inp, trajectory)

        # create a trajectory to the desired qpos  (linear interpolation)
        for i in range(int(trajectory.duration * 50)):
            q_desired, _, _ = trajectory.at_time(i / 50.0)
            await self.set("q_desired", q_desired)

        # await self.stabilize()

