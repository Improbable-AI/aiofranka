"""
Controller configurations: how a policy drives the robot, in one YAML file.

A file names the controller mode, its gains, the policy rate and the tool it is for:

    mode: osc
    tool: test panda hand               # Desk end-effector profile, or none (optional)
    ee_kp: [100, 100, 100, 30, 30, 30]  # x, y, z [N/m] and rotation [Nm/rad], or one value for all
    ee_kd: 20                           # [N s/m, Nm s/rad]
    null_kp: 9                          # null space [Nm/rad], one value or one per joint (default 1)
    null_kd: 6                          # [Nm s/rad] (default 1)
    null_target: home                   # 7 joint positions [rad], home, or current (the default): the
                                        # joint positions when activated, as switch("osc") keeps
    tcp: [0, 0, 0.1034]                 # TCP in the flange frame: a translation [m] or a 4x4 pose
                                        # (default: the flange)
    frequency: 50                       # policy rate [Hz], the rate of controller.set()

or

    mode: impedance
    tool: none
    kp: [64, 64, 64, 64, 32, 32, 32]    # [Nm/rad], one value or one per joint
    kd: [16, 16, 16, 16, 8, 8, 8]       # [Nm s/rad]
    frequency: 50

controller.activate("osc.yaml") applies it, and refuses if the tool is not the end
effector active in Desk (none is Desk's "No End Effector"; leave tool out to skip the
check), or if the arm is not at the null_target of an OSC configuration that sets one
(await controller.move(config["null_target"]) first). The system identification examples collect with it (--activate) and add what they
fit to its sim section, one entry per physics step: the gains and joint parameters with
which a simulation that runs the controller every physics step on fr3.xml's motors, with
the payload the fit assumed, responds as the robot did. The collectors also list the
recordings they made on the robot, newest last, which is what the fitters fit by default.

    recordings:
    - {path: ../examples/sysid_data/osc_sysid_<date>.npz, date: ..., tool: ...}

    sim:
    - physics_dt: 0.005
      ee_kp: [...]
      ...
      armature: [...]
      damping: [...]
      frictionloss: [...]
      payload: {mass: ..., com: [...], inertia: [...]}
"""

import math
import os
from pathlib import Path

import numpy as np
import yaml

# The home pose of controller.move() and of the system identification examples.
HOME = np.array([0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7854])

MODES = {
    "impedance": ("kp", "kd"),
    "osc": ("ee_kp", "ee_kd", "null_kp", "null_kd", "null_target", "tcp"),
}
COMMON = ("mode", "frequency", "tool", "name", "recordings", "sim")
# Sections the sysid examples write, kept at the end of the file in this order.
WRITTEN = ("recordings", "sim")


def load_config(source):
    """
    Read and check a controller configuration.

    Args:
        source (str | Path | dict): YAML file, or its contents

    Returns:
        dict: mode, frequency [Hz], recordings and sim (the lists the sysid examples write),
        plus kp and kd (7,)
        for impedance, or ee_kp and ee_kd (6,), null_kp and null_kd (7,), null_target (7,)
        or None for the joint positions at activation, and tcp (4, 4) for osc, and tool and
        name if given

    Raises:
        ValueError: If the file is not valid YAML, or the configuration is incomplete, has
            unknown keys or wrong values
    """
    if isinstance(source, (str, Path)):
        try:
            raw = yaml.safe_load(Path(source).read_text())
        except yaml.YAMLError as error:
            raise ValueError(f"{source} is not valid YAML: {error}") from None
        raw = {} if raw is None else raw
    else:
        raw = source
    if not isinstance(raw, dict):
        raise ValueError("A configuration maps keys such as mode and frequency to values")
    mode = raw.get("mode")
    if mode not in MODES:
        raise ValueError(f"mode must be one of {sorted(MODES)}, not {mode!r}")
    unknown = set(raw) - set(MODES[mode]) - set(COMMON)
    if unknown:
        raise ValueError(f"Unknown keys for mode {mode}: {sorted(unknown)}")

    def vector(key, size, default=None):
        if key not in raw:
            if default is None:
                raise ValueError(f"{key} is missing")
            return np.full(size, float(default))
        try:
            value = np.array(raw[key], dtype=float)
        except (TypeError, ValueError):
            raise ValueError(f"{key} takes numbers, not {raw[key]!r}") from None
        if value.ndim == 0:
            value = np.full(size, float(value))
        if value.shape != (size,):
            raise ValueError(f"{key} takes one value or {size}, not {raw[key]!r}")
        if not np.all(np.isfinite(value)):
            raise ValueError(f"{key} has no value or one that is not finite: {raw[key]!r}")
        return value

    try:
        frequency = float(raw["frequency"])
    except KeyError:
        raise ValueError("frequency (the policy rate in Hz) is missing") from None
    except (TypeError, ValueError):
        raise ValueError(f"frequency takes a number, not {raw['frequency']!r}") from None
    if not (math.isfinite(frequency) and frequency > 0):
        raise ValueError(f"frequency must be positive and finite, not {raw['frequency']!r}")
    config = {"mode": mode, "frequency": frequency}
    if mode == "impedance":
        gains = {"kp": vector("kp", 7), "kd": vector("kd", 7)}
    else:
        gains = {"ee_kp": vector("ee_kp", 6), "ee_kd": vector("ee_kd", 6),
                 "null_kp": vector("null_kp", 7, 1.0), "null_kd": vector("null_kd", 7, 1.0)}
        target = raw.get("null_target")
        if target is None or (isinstance(target, str) and target == "current"):
            config["null_target"] = None  # the joint positions when activated
        elif isinstance(target, str) and target == "home":
            config["null_target"] = HOME.copy()
        else:
            config["null_target"] = vector("null_target", 7)
        config["tcp"] = tcp_transform(raw.get("tcp", [0.0, 0.0, 0.0]))
    for key, value in gains.items():
        if np.any(value < 0):
            raise ValueError(f"{key} must not be negative, not {value.tolist()}")
    config.update(gains)

    if "tool" in raw:
        tool = raw["tool"]
        if not isinstance(tool, str) or not tool.strip():
            raise ValueError(f"tool takes the name of a Desk end-effector profile, or none; not {tool!r}. "
                             "Leave it out to skip the check.")
        config["tool"] = "none" if tool.strip().lower() == "none" else tool
    if raw.get("name") is not None:
        config["name"] = str(raw["name"])
    config["recordings"] = _recordings(raw.get("recordings"))
    config["sim"] = _sim_entries(raw.get("sim"))
    return config


def _recordings(recordings):
    """The recordings section as a list of entries, each with a path."""
    if recordings is None:
        return []
    if not isinstance(recordings, list) or not all(
            isinstance(entry, dict) and isinstance(entry.get("path"), str) for entry in recordings):
        raise ValueError("recordings takes a list of entries, each with a path")
    return [dict(entry) for entry in recordings]


def _sim_entries(sims):
    """The sim section as a list of entries, each with a numeric physics_dt."""
    if sims is None:
        return []
    if not isinstance(sims, list) or not all(isinstance(entry, dict) for entry in sims):
        raise ValueError("sim takes a list of entries, each with a physics_dt")
    out = []
    for entry in sims:
        try:
            physics_dt = float(entry["physics_dt"])
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"A sim entry needs a numeric physics_dt, not {entry.get('physics_dt')!r}") from None
        out.append(dict(entry, physics_dt=physics_dt))
    return out


def tcp_transform(value):
    """
    A TCP pose in the flange frame, from a translation (3,), 16 values row by row, or a 4x4 pose.

    Raises:
        ValueError: If it is none of these, or its rotation is not a rotation
    """
    try:
        transform = np.array(value, dtype=float)
    except (TypeError, ValueError):
        raise ValueError("transform must be a translation (3,) or a 4x4 pose with a rotation") from None
    if transform.shape == (3,):
        transform = np.block([[np.eye(3), transform[:, None]], [np.zeros(3), 1.0]])
    elif transform.shape == (16,):
        transform = transform.reshape(4, 4)
    rotation = transform[:3, :3] if transform.shape == (4, 4) else None
    if (rotation is None or not np.all(np.isfinite(transform)) or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6)
            or np.linalg.det(rotation) < 0):
        raise ValueError("transform must be a translation (3,) or a 4x4 pose with a rotation")
    return transform


def to_yaml(config):
    """The configuration as plain values, as written in a file (without recordings and sim)."""
    out = {}
    for key, value in config.items():
        if key in WRITTEN:
            continue
        out[key] = value.tolist() if isinstance(value, np.ndarray) else value
    if out.get("mode") == "osc" and out.get("null_target") is None:
        out["null_target"] = "current"
    return out


def same_controller(a, b):
    """Whether two configurations drive the robot the same way, with the same tool (sim and name aside)."""
    a, b = load_config(a), load_config(b)
    if (a["mode"] != b["mode"] or not math.isclose(a["frequency"], b["frequency"])
            or a.get("tool") != b.get("tool")):
        return False
    def same(x, y):
        if x is None or y is None:
            return x is None and y is None
        return np.allclose(x, y)

    return all(same(a[key], b[key]) for key in MODES[a["mode"]])


def _same_dt(a, b):
    # Physics steps are whole milliseconds; a float32 timestep is off by ~1e-10 s.
    return math.isclose(float(a), float(b), rel_tol=1e-6, abs_tol=1e-9)


def sim_entry(config, physics_dt):
    """
    The fitted simulation parameters for a physics step, as arrays.

    Args:
        config (str | Path | dict): Configuration, or its file
        physics_dt (float): Physics step of the simulation [s]

    Raises:
        KeyError: If the configuration has no entry for that physics step
    """
    sims = _sim_entries(config["sim"]) if isinstance(config, dict) and "sim" in config else load_config(config)["sim"]
    for entry in sims:
        if _same_dt(entry["physics_dt"], physics_dt):
            return {key: np.asarray(value) if isinstance(value, list) else value for key, value in entry.items()}
    known = [entry["physics_dt"] for entry in sims]
    raise KeyError(f"No sim entry for physics_dt {physics_dt}; the file has {known}")


def save_sim(path, entry, controller=None):
    """
    Add a fitted entry to the sim section of a configuration file, replacing one with
    the same physics_dt.

    The rest of the file stays as written, comments included; the recordings and sim
    sections move to its end. The new file is checked before it replaces the old one, so
    a failure leaves the file as it was.

    Args:
        path (str | Path): Configuration file
        entry (dict): physics_dt [s] and the fitted values (arrays or lists)
        controller (str | Path | dict | None): If given, the configuration the fit is for;
            the file must still describe the same controller

    Raises:
        ValueError: If the file is not a valid configuration, or no longer describes controller
        RuntimeError: If the sim section could not be rewritten without changing the rest
    """
    entry = _plain(entry)

    def update(config):
        sims = [s for s in config["sim"] if not _same_dt(s["physics_dt"], entry["physics_dt"])]
        return {"sim": sorted(sims + [entry], key=lambda s: s["physics_dt"])}

    _write_sections(path, update, controller, "add the fit")


def add_recording(path, recording, controller=None):
    """
    Add a recording collected with a configuration to its recordings section, newest last.

    Args:
        path (str | Path): Configuration file
        recording (dict): path of the recording (kept relative to the file's folder when
            it is under the same root) and what else to note, e.g. date and tool
        controller (str | Path | dict | None): If given, the configuration the recording
            was collected with; the file must still describe the same controller

    Raises:
        ValueError, RuntimeError: As save_sim()
    """
    path = Path(path)
    entry = dict(_plain(recording), path=_relative(Path(recording["path"]), path.parent))
    _write_sections(path, lambda config: {"recordings": config["recordings"] + [entry]}, controller,
                    "add the recording")


def latest_recording(path):
    """
    The newest recording in a configuration file's recordings section, or None.

    Args:
        path (str | Path): Configuration file

    Returns:
        Path | None: The recording, with a relative path resolved against the file's folder
    """
    recordings = load_config(path)["recordings"]
    if not recordings:
        return None
    recording = Path(recordings[-1]["path"]).expanduser()
    return recording if recording.is_absolute() else (Path(path).parent / recording).resolve()


def _relative(target, folder):
    try:
        return os.path.relpath(target.resolve(), folder.resolve())
    except ValueError:  # e.g. another drive
        return str(target.resolve())


def _write_sections(path, update, controller, action):
    """
    Rewrite the recordings and sim sections of a configuration file, as update(config)
    returns them, at the end of the file; keep the rest as written, check the result, and
    replace the file only then.
    """
    path = Path(path)
    text = path.read_text()
    config = load_config(path)
    if controller is not None and not same_controller(config, controller):
        raise ValueError(f"{path} no longer describes the controller to {action} for")
    sections = {key: config[key] for key in WRITTEN}
    sections.update(update(config))
    sections = {key: _plain(value) for key, value in sections.items() if value}

    # Cut the old sections out, located by the YAML parser: from their key to the end of
    # their last value (comments after them stay).
    lines = text.splitlines(keepends=True)
    cuts = []
    for key, value in yaml.compose(text).value:
        if key.value in WRITTEN:
            end = _content_end(value)
            cuts.append((key.start_mark.line, end.line + (end.column > 0)))
    for start, stop in sorted(cuts, reverse=True):
        lines = lines[:start] + lines[stop:]
    head = "".join(lines).rstrip()
    body = "\n".join(yaml.safe_dump({key: value}, sort_keys=False, default_flow_style=None, width=120)
                     for key, value in sections.items())
    new = f"{head}\n\n{body}" if head else body

    try:
        written = load_config(yaml.safe_load(new))
    except (yaml.YAMLError, ValueError) as error:
        raise RuntimeError(f"Could not {action} to {path} ({error}); the file was not changed") from None
    if (not same_controller(written, config)
            or [r["path"] for r in written["recordings"]] != [r["path"] for r in sections.get("recordings", [])]
            or [s["physics_dt"] for s in written["sim"]] != [s["physics_dt"] for s in sections.get("sim", [])]):
        raise RuntimeError(f"Could not {action} to {path} without changing the rest; the file was not changed")
    partial = path.with_name(f".{path.name}.partial")
    partial.write_text(new)
    os.replace(partial, path)


def _content_end(node):
    """Where the content of a YAML node ends: block collections end only at the next token."""
    if isinstance(node, yaml.ScalarNode) or getattr(node, "flow_style", False) or not node.value:
        return node.end_mark
    if isinstance(node, yaml.MappingNode):
        return max((_content_end(n) for pair in node.value for n in pair), key=lambda m: m.index)
    return max((_content_end(n) for n in node.value), key=lambda m: m.index)


def _plain(value):
    """Lists and Python numbers, rounded to 6 significant digits, for YAML."""
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(f"{float(value):.6g}")
    return value
