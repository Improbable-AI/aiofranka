"""
Toggle gripper open/closed with SpaceMouse left button.

Controls:
- Left SpaceMouse button: toggle gripper state
- Ctrl+C: quit

Backend selection:
- Robotiq: endpoint is serial port (e.g. /dev/ttyUSB1)
- Franka hand: endpoint is robot IP (e.g. 172.16.0.2)
"""

import argparse
import time

from aiofranka import GripperRemoteController
from aiofranka.utils.spacemouse import SpaceMouse


def main():
    parser = argparse.ArgumentParser(description="Toggle gripper with SpaceMouse left button")
    parser.add_argument(
        "--endpoint",
        type=str,
        default="172.16.0.2",
        help="Robotiq serial port or Franka robot IP",
    )
    parser.add_argument(
        "--backend",
        choices=["auto", "robotiq", "franka"],
        default="auto",
        help="Gripper backend (default: auto)",
    )
    parser.add_argument("--speed", type=int, default=200)
    parser.add_argument("--force", type=int, default=200)
    parser.add_argument("--home", action="store_true", help="Run homing on startup (Franka backend only)")
    parser.add_argument("--debounce", type=float, default=0.2, help="Toggle debounce in seconds")
    args = parser.parse_args()

    gripper = GripperRemoteController(
        args.endpoint,
        backend=args.backend,
        speed=args.speed,
        force=args.force,
        home=args.home,
    )
    gripper.start()

    spacemouse = SpaceMouse(yaw_only=True)

    is_closed = False
    gripper.open()
    gripper.wait_until_reached(timeout=3.0)

    print("[Init] SpaceMouse ready. Press left button to toggle gripper. Ctrl+C to exit.")
    print(f"[State] OPEN (qpos={gripper.qpos})")

    left_prev = False
    last_toggle_time = 0.0

    try:
        while True:
            _, _, buttons = spacemouse.read()

            left_now = False
            if buttons:
                try:
                    left_now = bool(buttons[0])
                except Exception:
                    left_now = False

            now = time.time()
            if left_now and not left_prev and (now - last_toggle_time) >= args.debounce:
                is_closed = not is_closed
                if is_closed:
                    gripper.close()
                    label = "CLOSED"
                else:
                    gripper.open()
                    label = "OPEN"
                last_toggle_time = now
                print(f"[Toggle] {label} (q_desired={gripper.q_desired}, qpos={gripper.qpos})")

            left_prev = left_now
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[Exit] Stopping...")
    finally:
        try:
            gripper.open()
            gripper.wait_until_reached(timeout=2.0)
        except Exception:
            pass
        gripper.stop()


if __name__ == "__main__":
    main()
