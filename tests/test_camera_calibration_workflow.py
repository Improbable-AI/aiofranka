"""Hardware-free checks for the browser workflow and offline fitting entry point."""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import importlib.util
import http.client
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

if cv2 is not None:
    script = Path(__file__).resolve().parents[1] / "examples" / "12_camera_calibration.py"
    spec = importlib.util.spec_from_file_location("camera_calibration_workflow", script)
    calibration = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(calibration)


class Browser:
    """Record which status the browser can observe before explicit shutdown."""

    port = 8080

    def __init__(self):
        self.closed = False
        self.status = ""
        self.result = None
        self.verification = None
        self.observations = []
        self.actions = iter([None, "quit"])

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.closed = True

    def set_status(self, status, **_):
        self.status = status

    def set_result(self, result, status):
        self.result, self.status = result, status

    def set_verification(self, directory):
        self.verification = directory

    def poll_action(self):
        self.observations.append((self.closed, self.result, self.status))
        return next(self.actions)


@unittest.skipIf(cv2 is None, "Camera calibration requires optional OpenCV")
class CalibrationWorkflowTest(unittest.TestCase):
    def setUp(self):
        self.output = io.StringIO()
        contexts = ExitStack()
        self.addCleanup(contexts.close)
        contexts.enter_context(redirect_stdout(self.output))
        contexts.enter_context(redirect_stderr(self.output))

    def test_default_fit_renders_after_cleanup_and_results_remain_until_quit(self):
        browser = Browser()
        args = SimpleNamespace(host="127.0.0.1", port=8080, command="calibrate", no_overlay=False)
        session = Path("/tmp/synthetic-calibration-session")
        result = {"T_base_camera": np.eye(4).tolist(), "metrics": {"held_out": {"rms_px": 0.35}}}
        cleanup = []

        def collected(*_):
            cleanup.append("hardware closed")
            return session

        def fitted(*_):
            self.assertEqual(cleanup, ["hardware closed"])
            self.assertFalse(browser.closed)
            self.assertIn("Fitting", browser.status)
            cleanup.append("fit saved")
            return session / "calibration.json", result

        def rendered(actual_args, actual_session, actual_result, actual_ui):
            self.assertEqual(cleanup, ["hardware closed", "fit saved"])
            self.assertIs(actual_args, args)
            self.assertEqual(actual_session, session)
            self.assertEqual(actual_result, session / "calibration.json")
            self.assertIs(actual_ui, browser)
            self.assertEqual(browser.result, result)
            self.assertFalse(browser.closed)
            return session / "verification"

        with mock.patch.object(calibration, "WebUI", return_value=browser, create=True), \
                mock.patch.object(calibration, "collect_samples", side_effect=collected), \
                mock.patch.object(calibration, "save_fit", side_effect=fitted), \
                mock.patch.object(calibration, "generate_overlays", side_effect=rendered) as render, \
                mock.patch.object(calibration.time, "sleep"):
            self.assertEqual(calibration.collect(args), 0)
            render.assert_called_once()
        self.assertTrue(browser.closed)
        self.assertEqual(browser.verification, session / "verification")
        self.assertEqual(len(browser.observations), 2)
        for closed, visible_result, status in browser.observations:
            self.assertFalse(closed)
            self.assertEqual(visible_result, result)
            self.assertIn("0.350", status)

    def test_capture_and_fit_errors_remain_visible_until_quit(self):
        for failure_stage in ("collect_samples", "save_fit"):
            with self.subTest(stage=failure_stage):
                browser = Browser()
                args = SimpleNamespace(host="127.0.0.1", port=8080, command="calibrate", no_overlay=False)
                with mock.patch.object(calibration, "WebUI", return_value=browser, create=True), \
                        mock.patch.object(calibration, "collect_samples", return_value=Path("/tmp/session")), \
                        mock.patch.object(calibration, "save_fit"), \
                        mock.patch.object(calibration.time, "sleep"):
                    getattr(calibration, failure_stage).side_effect = ValueError("Synthetic failure")
                    self.assertEqual(calibration.collect(args), 1)
                self.assertTrue(browser.closed)
                self.assertEqual(len(browser.observations), 2)
                for closed, result, status in browser.observations:
                    self.assertFalse(closed)
                    self.assertIsNone(result)
                    self.assertIn("Synthetic failure", status)
                    self.assertIn("retained", status)

    def test_camera_failure_stops_robot_and_preserves_session(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            config = directory / "cube.json"
            config.write_text("{}")
            args = SimpleNamespace(command="calibrate", cube_config=config,
                                   output=directory / "session", robot_ip="test-robot", damping=2.0)
            camera = mock.MagicMock()
            camera.__enter__.return_value = camera
            camera.metadata = {"serial": "synthetic"}
            camera.read.side_effect = [(np.zeros((4, 4, 3), dtype=np.uint8), {}),
                                       RuntimeError("PyCAAS stream unavailable")]
            controller = mock.MagicMock()
            fake_robot_module = SimpleNamespace(
                __file__=str(calibration.ROOT / "aiofranka" / "__init__.py"),
                FrankaRemoteController=mock.Mock(return_value=controller),
            )
            with mock.patch.object(calibration, "PycaasCamera", return_value=camera), \
                    mock.patch.object(calibration, "CubeDetector"), \
                    mock.patch.dict(sys.modules, {"aiofranka": fake_robot_module}):
                with self.assertRaisesRegex(RuntimeError, "PyCAAS stream unavailable"):
                    calibration.collect_samples(args, mock.MagicMock())
            controller.start.assert_called_once()
            controller.stop.assert_called_once()
            camera.__exit__.assert_called_once()
            self.assertEqual(json.loads((args.output / "views.json").read_text())["views"], [])
            self.assertEqual((args.output / "cube_config.json").read_text(), "{}")

    def saved_dataset(self, directory):
        session = directory / "views.json"
        session.write_text(json.dumps({
            "schema_version": 1, "setup": "fixed_camera_robot_held_cube", "views": [],
            "transform_convention": "T_A_B maps B to A", "ee_frame": "attachment_site",
            "camera": {"serial": "synthetic"}, "robot_model_sha256": "synthetic-model",
            "cube_config": {},
        }))
        result = {
            "T_base_camera": np.eye(4).tolist(),
            "metrics": {name: {"rms_px": 0.25, "view_count": 12}
                        for name in ("training", "held_out", "all_views")},
        }
        return session, result

    def test_overlay_failure_retains_saved_matrix_and_browser_result(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            _, result = self.saved_dataset(directory)
            browser = Browser()
            args = SimpleNamespace(host="127.0.0.1", port=8080, command="calibrate", no_overlay=False)
            with mock.patch.object(calibration, "WebUI", return_value=browser), \
                    mock.patch.object(calibration, "collect_samples", return_value=directory), \
                    mock.patch.object(calibration, "fit_calibration", return_value=result), \
                    mock.patch.object(calibration, "generate_overlays", side_effect=RuntimeError("EGL unavailable")), \
                    mock.patch.object(calibration.time, "sleep"):
                self.assertEqual(calibration.collect(args), 1)
            saved = json.loads((directory / "calibration.json").read_text())
            self.assertEqual(saved["T_base_camera"], np.eye(4).tolist())
            self.assertEqual(browser.result, saved)
            self.assertIsNone(browser.verification)
            for closed, visible_result, status in browser.observations:
                self.assertFalse(closed)
                self.assertEqual(visible_result["T_base_camera"], np.eye(4).tolist())
                self.assertIn("Overlay rendering failed", status)
                self.assertIn("still available", status)

    def test_no_overlay_skips_browser_rendering(self):
        browser = Browser()
        args = SimpleNamespace(host="127.0.0.1", port=8080, command="calibrate", no_overlay=True)
        session = Path("/tmp/synthetic-calibration-session")
        result = {"T_base_camera": np.eye(4).tolist(), "metrics": {"held_out": {"rms_px": 0.35}}}
        with mock.patch.object(calibration, "WebUI", return_value=browser), \
                mock.patch.object(calibration, "collect_samples", return_value=session), \
                mock.patch.object(calibration, "save_fit", return_value=(session / "calibration.json", result)), \
                mock.patch.object(calibration, "generate_overlays") as render, \
                mock.patch.object(calibration.time, "sleep"):
            self.assertEqual(calibration.collect(args), 0)
            render.assert_not_called()
        self.assertEqual(browser.result, result)
        self.assertIsNone(browser.verification)

    def test_offline_fit_with_and_without_overlays_needs_no_camera(self):
        for skip_overlay in (False, True):
            with self.subTest(no_overlay=skip_overlay), tempfile.TemporaryDirectory() as directory:
                directory = Path(directory)
                session, result = self.saved_dataset(directory)
                argv = [str(script), "fit", "--session", str(session)]
                if skip_overlay:
                    argv.append("--no-overlay")
                with mock.patch.object(sys, "argv", argv), \
                        mock.patch.object(calibration, "fit_calibration", return_value=result), \
                        mock.patch.object(calibration, "PycaasCamera") as camera, \
                        mock.patch.object(calibration, "WebUI") as browser, \
                        mock.patch.object(calibration, "generate_overlays") as render, \
                        mock.patch.dict(sys.modules, {"aiofranka": None, "pyrealsense2": None,
                                                     "pycaas": None, "aprilcube": None}):
                    self.assertEqual(calibration.main(), 0)
                    camera.assert_not_called()
                    browser.assert_not_called()
                    if skip_overlay:
                        render.assert_not_called()
                    else:
                        render.assert_called_once()
                        self.assertTrue((directory / "calibration.json").is_file())
                saved = json.loads((directory / "calibration.json").read_text())
                self.assertEqual(saved["T_base_camera"], np.eye(4).tolist())
                self.assertEqual(saved["dataset"], str(session))
                self.assertEqual(len(saved["dataset_sha256"]), 64)

    def test_overlay_process_uses_current_python_egl_and_saved_session(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            session = directory / "views.json"
            session.write_text("{}")
            output = directory / "verification_refit"
            output.mkdir()
            (output / "calibration_summary.jpg").write_bytes(b"synthetic summary")
            args = SimpleNamespace(cube_xml=directory / "custom_cube.xml", cube_config=directory / "config.json")
            process = mock.Mock(stdout=io.StringIO("Rendered 1/1: view 0\nSaved overlays\n"))
            process.wait.return_value = process.poll.return_value = 0
            browser = Browser()
            with mock.patch.object(calibration.subprocess, "Popen", return_value=process) as start, \
                    mock.patch.dict(calibration.os.environ, {}, clear=True):
                self.assertEqual(calibration.generate_overlays(args, session, directory / "refit.json", browser), output)
            command = start.call_args.args[0]
            self.assertEqual(command[:2], [sys.executable, str(script.with_name("13_verify_camera_calibration.py"))])
            self.assertEqual(command[command.index("--session") + 1], str(directory))
            self.assertEqual(command[command.index("--calibration") + 1], str(directory / "refit.json"))
            self.assertEqual(command[command.index("--cube-xml") + 1], str(args.cube_xml))
            self.assertIn("--no-serve", command)
            self.assertEqual(start.call_args.kwargs["env"]["MUJOCO_GL"], "egl")
            self.assertTrue(process.stdout.closed)
            self.assertIn("Saved overlays", browser.status)

    def test_overlay_process_failure_or_missing_summary_is_reported(self):
        for returncode, diagnostic in ((3, "EGL unavailable"), (0, "did not produce calibration_summary.jpg")):
            with self.subTest(returncode=returncode), tempfile.TemporaryDirectory() as directory:
                directory = Path(directory)
                args = SimpleNamespace(cube_xml=directory / "cube.xml", cube_config=directory / "config.json")
                process = mock.Mock(stdout=io.StringIO("EGL unavailable\n" if returncode else "Export complete\n"))
                process.wait.return_value = process.poll.return_value = returncode
                with mock.patch.object(calibration.subprocess, "Popen", return_value=process):
                    with self.assertRaisesRegex(RuntimeError, diagnostic):
                        calibration.generate_overlays(args, directory, directory / "calibration.json")
                self.assertTrue(process.stdout.closed)

    def test_browser_serves_matrix_and_overlay_within_export_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            verification = directory / "verification"
            (verification / "0000").mkdir(parents=True)
            (verification / "index.html").write_text("<h1>Saved comparison</h1>")
            ok, encoded = cv2.imencode(".jpg", np.zeros((8, 8, 3), dtype=np.uint8))
            self.assertTrue(ok)
            (verification / "0000/comparison.jpg").write_bytes(encoded.tobytes())
            (verification / "report.json").write_text(json.dumps({
                "summary": {"view_index": 0},
                "views": [{"files": {"comparison": "0000/comparison.jpg"}}],
            }))
            (directory / "outside.json").write_text('{"private":true}')
            result = {"T_base_camera": np.eye(4).tolist()}
            with calibration.WebUI("127.0.0.1", 0) as ui:
                ui.set_result(result, "Calibration ready")
                ui.set_verification(verification)
                connection = http.client.HTTPConnection("127.0.0.1", ui.port, timeout=3)
                try:
                    def get(route):
                        connection.request("GET", route)
                        response = connection.getresponse()
                        return response.status, response.read()

                    status, body = get("/status")
                    self.assertEqual(status, 200)
                    state = json.loads(body)
                    self.assertEqual(state["T_base_camera"], result["T_base_camera"])
                    self.assertTrue(state["result_ready"])
                    self.assertTrue(state["verification_ready"])
                    self.assertEqual(get("/calibration.json")[1], (json.dumps(result, indent=2) + "\n").encode())
                    self.assertEqual(get(state["overlay_url"]), (200, encoded.tobytes()))
                    self.assertEqual(get("/verification/")[0], 200)
                    self.assertIn(b'id="matrix"', get("/")[1])
                    self.assertEqual(get("/verification/%2e%2e/outside.json")[0], 404)
                finally:
                    connection.close()


if __name__ == "__main__":
    unittest.main()
