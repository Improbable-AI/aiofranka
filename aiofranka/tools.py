"""
Tools on the flange, kept as end-effector profiles in Desk.

Desk keeps named end-effector profiles (Settings > End Effector): the mass, center
of mass and inertia of the tool on the flange, and its tool center point. The robot
compensates the gravity of the active profile, and RobotInterface merges it into
the MuJoCo model when it connects. These functions manage the profiles through the
admin API that the Desk web UI uses, so Desk stays the one place that knows which
tool is mounted.

Example:
    >>> estimate = await controller.identify_payload()
    >>> await controller.stop()
    >>> aiofranka.save_tool("gripper", estimate.mass, estimate.com)
    >>> aiofranka.load_tool("gripper")
"""

from dataclasses import dataclass, replace

import numpy as np

# Desk's device for tools other than the Franka Hand ("Generic Device").
GENERIC_DEVICE = "ee-no-gripper"

# Desk's built-in profile without an end effector.
NO_END_EFFECTOR = "ee-no-profile"

# Radius of the solid sphere whose inertia a new tool gets if none is given [m].
DEFAULT_RADIUS = 0.05


@dataclass
class Tool:
    """
    An end-effector profile in Desk.

    Attributes:
        name (str): Name of the profile
        mass (float): Mass [kg]
        com (np.ndarray): Center of mass in the flange frame [m] (3,)
        inertia (np.ndarray): Inertia about the center of mass in the flange
            frame [kg m^2] (3, 3)
        translation (np.ndarray): Translation from the flange to the tool center
            point [m] (3,)
        rotation (np.ndarray): Rotation from the flange to the tool center point
            as roll, pitch, yaw [rad] (3,)
        device (str): Desk device, "ee-no-gripper" (Generic Device) or
            "ee-gripper" (Franka Hand)
        active (bool): Whether the robot uses this profile
        id (str | None): Id of the profile in Desk
    """

    name: str
    mass: float
    com: np.ndarray
    inertia: np.ndarray
    translation: np.ndarray
    rotation: np.ndarray
    device: str = GENERIC_DEVICE
    active: bool = False
    id: str = None


def _from_desk(profile):
    inertial, tcp = profile["inertial"], profile["tcp"]
    c, i = inertial["centerOfMass"], inertial["inertia"]
    t, r = tcp["translation"], tcp["rotation"]
    return Tool(
        name=profile["name"],
        mass=float(inertial["mass"]),
        com=np.array([c["x"], c["y"], c["z"]], dtype=float),
        inertia=np.array([
            [i["x11"], i["x12"], i["x13"]],
            [i["x12"], i["x22"], i["x23"]],
            [i["x13"], i["x23"], i["x33"]],
        ], dtype=float),
        translation=np.array([t["x"], t["y"], t["z"]], dtype=float),
        rotation=np.array([r["roll"], r["pitch"], r["yaw"]], dtype=float),
        device=profile["deviceId"],
        active=bool(profile.get("active", False)),
        id=profile.get("id"),
    )


def _to_desk(tool):
    c, i, t, r = tool.com, tool.inertia, tool.translation, tool.rotation
    return {
        "name": tool.name,
        "deviceId": tool.device,
        "inertial": {
            "mass": float(tool.mass),
            "centerOfMass": {"x": float(c[0]), "y": float(c[1]), "z": float(c[2])},
            "inertia": {
                "x11": float(i[0, 0]), "x12": float(i[0, 1]), "x13": float(i[0, 2]),
                "x22": float(i[1, 1]), "x23": float(i[1, 2]), "x33": float(i[2, 2]),
            },
        },
        "tcp": {
            "translation": {"x": float(t[0]), "y": float(t[1]), "z": float(t[2])},
            "rotation": {"roll": float(r[0]), "pitch": float(r[1]), "yaw": float(r[2])},
        },
    }


def _check(tool):
    """Raise ValueError if Desk or the robot would reject the tool."""
    if not tool.name:
        raise ValueError("A tool needs a name")
    if not 0 <= tool.mass <= 3:
        raise ValueError(f"mass must be in [0, 3] kg, not {tool.mass:.3f}")
    for label, values, bound in (("com", tool.com, 0.3), ("translation", tool.translation, 0.3),
                                 ("rotation", tool.rotation, np.pi), ("inertia", tool.inertia, 1.0)):
        if np.any(np.abs(values) > bound):
            raise ValueError(f"{label} must be within +/-{bound:g}, not {np.round(values, 4).tolist()}")
    if not np.allclose(tool.inertia, tool.inertia.T):
        raise ValueError("inertia must be symmetric")
    if tool.mass > 0 and np.linalg.eigvalsh(tool.inertia).min() <= 0:
        raise ValueError("A tool with mass needs a positive definite inertia")


class _Desk:
    """Session with the admin API of the Desk web UI."""

    def __init__(self, ip=None, username=None, password=None, protocol="https"):
        from aiofranka.server import _DeskClient, _DeskClientV2, _resolve_from_config

        self.ip, username, password = _resolve_from_config(ip, username, password)
        self._admin = _DeskClient(self.ip, username, password, protocol=protocol)
        self._admin.login()
        self._spoc = _DeskClientV2(self.ip, username, password, protocol=protocol)
        self._took_token = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        if self._took_token:
            self._spoc.release_token(best_effort=True)

    def _token(self):
        """The control token: the one saved by aiofranka.unlock(), or a new one."""
        from aiofranka.server import _load_token_state

        if self._spoc._token is None:
            self._spoc._token, self._spoc._token_id = _load_token_state(self.ip)
            if self._spoc._token is not None and not self._spoc.validate_token():
                self._spoc._token = None
            if self._spoc._token is None:
                self._spoc._with_retry(lambda: self._spoc.take_token(timeout=15), context="take_token")
                self._took_token = True
        return self._spoc._token

    def request(self, method, path, **kwargs):
        """Request /admin/api{path}, with the control token if Desk wants one."""
        r = self._admin._req(method, f"/admin/api{path}", **kwargs)
        if r.status_code == 423 or (r.status_code == 400 and "token" in r.text.lower()):
            r = self._admin._req(method, f"/admin/api{path}",
                                 headers={"X-Control-Token": self._token()}, **kwargs)
        if r.status_code not in (200, 201, 204):
            raise RuntimeError(f"Desk rejected {method} {path}: {r.status_code} {r.text[:300]}")
        return r

    def tools(self):
        return [_from_desk(p) for p in self.request("GET", "/end-effector/profiles").json()]

    def find(self, name, missing_ok=False):
        """The profile with that name, or None if missing_ok and there is none."""
        tools = self.tools()
        matches = [tool for tool in tools if tool.name == name]
        if len(matches) > 1:
            raise KeyError(f"Desk has {len(matches)} end-effector profiles named {name!r}")
        if not matches and not missing_ok:
            known = ", ".join(tool.name for tool in tools)
            raise KeyError(f"No end-effector profile named {name!r} in Desk. Profiles: {known}")
        return matches[0] if matches else None


def list_tools(ip=None, username=None, password=None, protocol="https"):
    """
    The end-effector profiles in Desk.

    Args:
        ip (str | None): Robot IP (default: from config or 172.16.0.2)
        username, password, protocol: Desk credentials (default: from config)

    Returns:
        dict: The tools by name, in Desk's order
    """
    with _Desk(ip, username, password, protocol) as desk:
        return {tool.name: tool for tool in desk.tools()}


def save_tool(name, mass, com, inertia=None, translation=None, rotation=None,
              ip=None, username=None, password=None, protocol="https"):
    """
    Save a tool as an end-effector profile in Desk.

    Updates the profile with that name, keeping what is not given, or creates one
    for a Generic Device. Does not activate it; see load_tool().

    Args:
        name (str): Name of the profile
        mass (float): Mass [kg]
        com (array-like): Center of mass in the flange frame [m] (3,)
        inertia (array-like | None): Inertia about the center of mass in the
            flange frame [kg m^2], as a (3, 3) tensor or its diagonal (3,).
            Default: keep the profile's, or for a new one that of a solid sphere
            of radius DEFAULT_RADIUS, since the robot needs some inertia.
        translation (array-like | None): Translation from the flange to the tool
            center point [m] (3,) (default: keep the profile's, or zero)
        rotation (array-like | None): Rotation from the flange to the tool center
            point as roll, pitch, yaw [rad] (3,) (default: keep the profile's, or
            zero)
        ip (str | None): Robot IP (default: from config or 172.16.0.2)
        username, password, protocol: Desk credentials (default: from config)

    Returns:
        Tool: The saved tool

    Raises:
        ValueError: If Desk or the robot would reject the tool
    """
    with _Desk(ip, username, password, protocol) as desk:
        existing = desk.find(name, missing_ok=True)
        if existing is not None and existing.id == NO_END_EFFECTOR:
            raise ValueError(f"{name!r} is built into Desk")

        base = existing or Tool(name, 0.0, np.zeros(3), np.zeros((3, 3)), np.zeros(3), np.zeros(3))
        tool = replace(base, mass=float(mass), com=np.asarray(com, dtype=float))
        if inertia is not None:
            inertia = np.asarray(inertia, dtype=float)
            tool.inertia = np.diag(inertia) if inertia.shape == (3,) else inertia
        elif tool.mass > 0 and np.linalg.eigvalsh(tool.inertia).min() <= 0:
            tool.inertia = 0.4 * tool.mass * DEFAULT_RADIUS ** 2 * np.eye(3)
        if translation is not None:
            tool.translation = np.asarray(translation, dtype=float)
        if rotation is not None:
            tool.rotation = np.asarray(rotation, dtype=float)
        _check(tool)

        if existing is None:
            tool.id = desk.request("POST", "/end-effector/profiles", json=_to_desk(tool)).json()["id"]
        else:
            desk.request("PUT", f"/end-effector/profiles/{existing.id}", json=_to_desk(tool))
        return tool


def load_tool(name, ip=None, username=None, password=None, protocol="https"):
    """
    Activate an end-effector profile in Desk, so the robot compensates that tool.

    Desk keeps it active across connections and reboots. RobotInterface merges it
    into the MuJoCo model when it connects, so connect after loading.

    Args:
        name (str): Name of the profile
        ip (str | None): Robot IP (default: from config or 172.16.0.2)
        username, password, protocol: Desk credentials (default: from config)

    Returns:
        Tool: The loaded tool

    Raises:
        KeyError: If Desk has no profile, or several, with that name
    """
    with _Desk(ip, username, password, protocol) as desk:
        return _activate(desk, desk.find(name))


def unload_tool(ip=None, username=None, password=None, protocol="https"):
    """
    Activate Desk's built-in profile without an end effector ("No End Effector").

    Args:
        ip (str | None): Robot IP (default: from config or 172.16.0.2)
        username, password, protocol: Desk credentials (default: from config)

    Returns:
        Tool: The built-in profile, now active
    """
    with _Desk(ip, username, password, protocol) as desk:
        tool = next((tool for tool in desk.tools() if tool.id == NO_END_EFFECTOR), None)
        if tool is None:
            raise KeyError(f"Desk has no built-in profile {NO_END_EFFECTOR!r}")
        return _activate(desk, tool)


def _activate(desk, tool):
    desk.request("PUT", "/end-effector/profiles/active", json=tool.id)
    active = _from_desk(desk.request("GET", "/end-effector/profiles/active").json())
    if active.id != tool.id:
        raise RuntimeError(f"Desk did not activate {tool.name!r}; {active.name!r} is active")
    tool.active = True
    return tool


def remove_tool(name, ip=None, username=None, password=None, protocol="https"):
    """
    Delete an end-effector profile from Desk.

    Raises:
        KeyError: If Desk has no profile, or several, with that name
        ValueError: If the profile is built in or active
    """
    with _Desk(ip, username, password, protocol) as desk:
        tool = desk.find(name)
        if tool.id == NO_END_EFFECTOR:
            raise ValueError(f"{name!r} is built into Desk")
        if tool.active:
            raise ValueError(f"{name!r} is active; load another tool first")
        desk.request("DELETE", f"/end-effector/profiles/{tool.id}")
