"""Isolated server startup/diagnostic checks; no robot, scheduling, or IPC calls."""

import ast
import asyncio
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np


SOURCE = Path(__file__).resolve().parents[1] / "aiofranka/server.py"
NAMES = {"ControlPreparationError", "_select_rt_cpu", "_configure_realtime", "ServerController", "_run_server"}
TREE = ast.parse(SOURCE.read_text())
ISOLATED = compile(ast.Module([node for node in TREE.body if getattr(node, "name", None) in NAMES],
                             type_ignores=[]), str(SOURCE), "exec")


class FakeBaseController:
    def __init__(self, robot):
        self.robot, self.task, self.error_callback = robot, None, None
        self.running, self.type = False, "impedance"
        self.q_desired = self.initial_qpos = np.zeros(7)
        self.ee_desired = self.initial_ee = np.eye(4)


class ServerRealtimeTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.events = []
        self.fake_os = SimpleNamespace(
            environ={}, cpu_count=lambda: 20,
            sched_getaffinity=mock.Mock(return_value=set(range(20))),
            sched_setaffinity=mock.Mock(side_effect=lambda *_: self.events.append("affinity")),
            sched_setscheduler=mock.Mock(side_effect=lambda *_: self.events.append("fifo")),
            sched_param=lambda priority: SimpleNamespace(sched_priority=priority), SCHED_FIFO=1,
            unlink=mock.Mock(),
        )
        self.logger = mock.Mock()
        self.loop = SimpleNamespace(add_signal_handler=mock.Mock(), run_in_executor=self.in_executor)

        async def sleep(_duration):
            await asyncio.sleep(0)

        self.namespace = {
            "FrankaController": FakeBaseController, "RobotInterface": mock.Mock(),
            "StateBlock": mock.Mock(), "np": np, "logger": self.logger, "os": self.fake_os,
            "open": mock.mock_open(read_data="0-11\n"),
            "asyncio": SimpleNamespace(get_event_loop=lambda: self.loop,
                                       wait_for=asyncio.wait_for, create_task=asyncio.create_task,
                                       sleep=sleep, TimeoutError=asyncio.TimeoutError,
                                       CancelledError=asyncio.CancelledError),
            "time": SimpleNamespace(perf_counter=mock.Mock()),
            "mujoco": SimpleNamespace(mj_forward=mock.Mock(), mj_jacSite=mock.Mock(), mj_fullM=mock.Mock()),
        }
        exec(ISOLATED, self.namespace)
        self.robot = SimpleNamespace(model=SimpleNamespace(nq=7, nv=7, nu=7), real=True,
                                     start=mock.Mock(side_effect=lambda: self.events.append("torque")))
        self.shm = mock.Mock()
        self.controller = self.namespace["ServerController"](self.robot, self.shm)
        self.controller._warmup_control_math = mock.Mock(side_effect=self.warmup)

    def in_executor(self, _executor, function):
        future = asyncio.get_running_loop().create_future()
        try:
            future.set_result(function())
        except Exception as exc:
            future.set_exception(exc)
        return future

    def warmup(self):
        self.events.append("warmup")
        return {"iterations": 100}

    def test_prefers_highest_allowed_performance_core(self):
        self.fake_os.sched_getaffinity.return_value = {2, 4, 8, 11, 19}
        self.namespace["open"] = mock.mock_open(read_data="0-3,8,10-11\n")
        self.assertEqual(self.namespace["_select_rt_cpu"](), 11)
        self.fake_os.sched_getaffinity.return_value = {2, 8, 19}
        self.assertEqual(self.namespace["_select_rt_cpu"](), 8)

    def test_falls_back_when_no_performance_cpu_is_available(self):
        for error in (FileNotFoundError(), PermissionError()):
            self.namespace["open"] = mock.Mock(side_effect=error)
            self.assertEqual(self.namespace["_select_rt_cpu"](), 19)
        self.namespace["open"] = mock.mock_open(read_data="0-11\n")
        self.fake_os.sched_getaffinity.return_value = {12, 19}
        self.assertEqual(self.namespace["_select_rt_cpu"](), 19)

    def test_explicit_override_must_be_in_allowed_affinity(self):
        self.fake_os.environ["AIOFRANKA_RT_CPU"] = "19"
        self.assertEqual(self.namespace["_select_rt_cpu"](), 19)
        for value in ("not-a-cpu", "-1", "20"):
            with self.subTest(value=value):
                self.fake_os.environ["AIOFRANKA_RT_CPU"] = value
                with self.assertRaisesRegex(ValueError, "AIOFRANKA_RT_CPU"):
                    self.namespace["_select_rt_cpu"]()
        self.fake_os.environ.clear()
        self.fake_os.sched_getaffinity.return_value = set()
        with self.assertRaisesRegex(ValueError, "No CPUs"):
            self.namespace["_select_rt_cpu"]()

    def test_scheduling_permission_failures_are_reported_before_start(self):
        self.fake_os.sched_setaffinity.side_effect = PermissionError("restricted")
        self.fake_os.sched_setscheduler.side_effect = PermissionError("restricted")
        self.assertEqual(self.namespace["_configure_realtime"](), 11)
        self.assertEqual(self.logger.warning.call_count, 2)
        self.fake_os.sched_setscheduler.assert_called_once()

    async def test_affinity_fifo_warmup_and_logs_finish_before_torque_start(self):
        async def run():
            self.events.append("loop")
            self.controller.running = True

        def start_robot():
            self.assertEqual(self.events, ["affinity", "fifo", "warmup"])
            self.events.append("torque")
            self.log_count_at_start = len(self.logger.method_calls)

        self.robot.start.side_effect = start_robot
        self.controller._run = run
        task = await self.controller.start()
        await task
        self.assertEqual(self.events, ["affinity", "fifo", "warmup", "torque", "loop"])
        self.assertEqual(len(self.logger.method_calls), self.log_count_at_start)
        self.fake_os.sched_setaffinity.assert_called_once_with(0, {11})
        scheduler_args = self.fake_os.sched_setscheduler.call_args.args
        self.assertEqual(scheduler_args[:2], (0, self.fake_os.SCHED_FIFO))
        self.assertEqual(scheduler_args[2].sched_priority, 80)

    async def test_warmup_failure_never_opens_torque_stream(self):
        original = ValueError("warmup failed")
        self.controller._warmup_control_math.side_effect = original
        with self.assertRaisesRegex(self.namespace["ControlPreparationError"], "warmup failed") as failure:
            await self.controller.start()
        self.assertIs(failure.exception.__cause__, original)
        self.robot.start.assert_not_called()
        self.assertIsNone(self.controller.task)

    async def test_invalid_cpu_never_warms_up_or_opens_torque_stream(self):
        self.fake_os.environ["AIOFRANKA_RT_CPU"] = "99"
        with self.assertRaisesRegex(self.namespace["ControlPreparationError"], "outside allowed") as failure:
            await self.controller.start()
        self.assertIsInstance(failure.exception.__cause__, ValueError)
        self.controller._warmup_control_math.assert_not_called()
        self.robot.start.assert_not_called()
        self.fake_os.sched_setscheduler.assert_not_called()

    async def test_failed_step_records_elapsed_and_last_received_success_rate_once(self):
        self.namespace["time"].perf_counter.side_effect = [10., 10.001, 11., 11.004]
        calls = 0

        def step():
            nonlocal calls
            calls += 1
            if calls > 1:
                raise RuntimeError("communication fault")
            self.controller._last_robot_mode = "RobotMode.Idle"
            self.controller._last_control_command_success_rate = .982

        self.controller.step = mock.Mock(side_effect=step)
        await self.controller._run()
        self.assertEqual((self.controller._completed_steps, self.controller._step_attempts), (1, 2))
        self.assertAlmostEqual(self.controller._last_step_elapsed_s, .004)
        self.assertAlmostEqual(self.controller._max_step_elapsed_s, .004)
        self.assertFalse(self.controller.running)
        self.shm.write_error.assert_called_once_with("communication fault")
        self.logger.error.assert_called_once()
        rendered = self.logger.error.call_args.args[0] % self.logger.error.call_args.args[1:]
        self.assertIn("controller=impedance", rendered)
        self.assertIn("includes robot read wait", rendered)
        self.assertIn("last_control_command_success_rate=0.982", rendered)
        self.logger.info.assert_not_called()
        self.logger.warning.assert_not_called()
        self.fake_os.sched_setaffinity.assert_not_called()

    async def test_healthy_loop_never_logs_each_step(self):
        self.namespace["time"].perf_counter.side_effect = [10., 10.001]
        self.controller.step = lambda: setattr(self.controller, "running", False)
        await self.controller._run()
        self.assertEqual(self.controller._completed_steps, 1)
        self.assertEqual(self.logger.method_calls, [])
        self.shm.write_error.assert_not_called()

    def test_step_retains_robot_mode_and_command_success_rate_from_real_state(self):
        state = SimpleNamespace(q=np.zeros(7), dq=np.zeros(7), tau_J_d=np.zeros(7),
                                robot_mode="RobotMode.Move", control_command_success_rate=.973)
        self.robot.torque_controller = SimpleNamespace(readOnce=mock.Mock(return_value=(state, None)))
        self.robot.site_id = 0
        self.robot.data = SimpleNamespace(qpos=np.zeros(7), qvel=np.zeros(7), ctrl=np.zeros(7),
                                         site=lambda _: SimpleNamespace(xmat=np.eye(3), xpos=np.zeros(3)))
        self.controller._impedance_step = mock.Mock()
        self.controller.step()
        self.assertEqual(self.controller._last_robot_mode, "RobotMode.Move")
        self.assertEqual(self.controller._last_control_command_success_rate, .973)
        self.logger.error.assert_not_called()
        del state.robot_mode, state.control_command_success_rate
        self.controller.step()
        self.assertIsNone(self.controller._last_robot_mode)
        self.assertIsNone(self.controller._last_control_command_success_rate)

    def server_environment(self):
        self.namespace.update({
            "_write_pid": mock.Mock(), "_write_progress": mock.Mock(), "_remove_pid": mock.Mock(),
            "atexit": SimpleNamespace(register=mock.Mock()),
            "signal": SimpleNamespace(SIGTERM=15, SIGINT=2),
            "threading": SimpleNamespace(Thread=mock.Mock()),
            "CommandHandler": mock.Mock(), "STATUS_RUNNING": 1, "STATUS_STOPPED": 0,
            "zmq_endpoint_for_ip": lambda _: "ipc:///tmp/never-created.sock",
        })
        self.robot.stop = mock.Mock()
        self.namespace["RobotInterface"].return_value = self.robot
        self.namespace["StateBlock"].return_value = self.shm
        self.controller.move = mock.AsyncMock()
        self.namespace["CommandHandler"].return_value._should_stop = False
        return mock.Mock(return_value=self.controller)

    async def test_startup_loop_failure_cannot_publish_running_home_or_retry(self):
        factory = self.server_environment()

        async def fail_during_start():
            self.controller._last_error = "communication fault during startup wait"
            self.controller.running = False
            self.shm.write_error(self.controller._last_error)
            return None

        self.controller.start = mock.AsyncMock(side_effect=fail_during_start)
        await self.namespace["_run_server"]("fake-host", unlock=False, home=True, controller_cls=factory)
        self.controller.start.assert_awaited_once()
        self.controller.move.assert_not_called()
        factory.assert_called_once()
        self.assertNotIn(mock.call(1), self.shm.write_status.call_args_list)
        self.assertEqual(self.controller._last_error, "communication fault during startup wait")

    async def test_preparation_failure_never_retries_or_recreates_robot(self):
        factory = self.server_environment()
        self.controller._warmup_control_math.side_effect = ValueError("warmup failed")
        await self.namespace["_run_server"]("fake-host", unlock=False, home=True, controller_cls=factory)
        self.controller._warmup_control_math.assert_called_once()
        self.robot.start.assert_not_called()
        self.namespace["RobotInterface"].assert_called_once_with("fake-host")
        factory.assert_called_once()
        self.controller.move.assert_not_called()
        self.assertNotIn(mock.call(1), self.shm.write_status.call_args_list)
        self.shm.write_error.assert_called_once_with("Control preparation failed: warmup failed")


if __name__ == "__main__":
    unittest.main()
