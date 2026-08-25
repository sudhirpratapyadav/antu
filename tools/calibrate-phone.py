#!/usr/bin/env python3
"""
Work out how the phone is mounted on the robot, by driving it.

No ruler and no jig. Drive a known pattern and compare what ARCore saw against
what the wheels reported; the transform that reconciles them is the mounting.
This is hand-eye calibration, and it is more accurate than measuring anyway,
because it captures where the *camera* is rather than where the phone case is.

Two motions, each answering a different question:

  spin in place  ->  how the rotation axes relate, and how far the camera sits
                     from the turning centre (it traces an arc)
  drive forward  ->  the scale, whether the phone is level, and which way it
                     faces relative to the robot

    ./tools/calibrate-phone.py <host>

Drive it somewhere with about a metre of clear space ahead. The robot moves.
"""
import math
import sys
import time
from importlib.machinery import SourceFileLoader
import os

HERE = os.path.dirname(os.path.abspath(__file__))
Ws = SourceFileLoader('wsc', os.path.join(HERE, 'ws-client.py')).load_module().Ws

SPIN_RATE = 0.4       # rad/s
SPIN_SECONDS = 4.0
FORWARD_SPEED = 0.15  # m/s
FORWARD_SECONDS = 2.0
SETTLE = 1.5          # let both estimators catch up before and after each move


class Session:
    def __init__(self, host):
        self.ws = Ws(host)
        self.ws.recv()                       # catalogue
        self.ws.send({"type": "subscribe",
                      "payload": {"channels": ["base.odom", "ar.pose"], "maxHz": 10}})
        self.state = {}

    def pump(self, secs, drive=None):
        end = time.time() + secs
        while time.time() < end:
            if drive is not None:
                # Repeated, so the arbiter keeps hold and the base does not time out.
                self.ws.send({"type": "drive", "payload": drive})
            self.ws.sock.settimeout(0.12)
            try:
                msg = self.ws.recv()
                if msg and msg["type"] == "msg":
                    self.state[msg["payload"]["channel"]] = msg["payload"]["value"]
            except Exception:
                pass
            time.sleep(0.05)

    def snap(self):
        odom, ar = self.state.get("base.odom"), self.state.get("ar.pose")
        if not odom or not ar:
            return None
        p, q = ar["pose"]["position"], ar["pose"]["rotation"]
        return {
            "x": odom["pose"]["x"], "y": odom["pose"]["y"], "th": odom["pose"]["theta"],
            "ar": (p["x"], p["y"], p["z"]),
            "yawY": yaw_about_y(q),
            "state": ar["state"],
        }

    def stop(self):
        self.ws.send({"type": "drive", "payload": {"linearX": 0, "angular": 0}})

    def close(self):
        self.stop()
        self.ws.close()


def yaw_about_y(q):
    """ARCore's world frame is y-up, so heading is rotation about y."""
    return math.atan2(2 * (q["w"] * q["y"] + q["z"] * q["x"]),
                      1 - 2 * (q["y"] * q["y"] + q["x"] * q["x"]))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def require_tracking(snap, what):
    if snap is None:
        raise SystemExit("no data yet — is the base connected and ARCore running?")
    if snap["state"] != "TRACKING":
        raise SystemExit(f"ARCore is {snap['state']} {what}; calibration needs "
                         "continuous tracking. Try somewhere with more texture.")


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    s = Session(host)
    results = {}
    try:
        s.pump(SETTLE)

        # ---- spin in place -------------------------------------------------
        print(f"spinning at {SPIN_RATE} rad/s for {SPIN_SECONDS}s ...")
        before = s.snap()
        require_tracking(before, "before the spin")
        s.pump(SPIN_SECONDS, {"linearX": 0.0, "angular": SPIN_RATE})
        s.stop()
        s.pump(SETTLE)
        after = s.snap()
        require_tracking(after, "after the spin")

        d_odom = wrap(after["th"] - before["th"])
        d_ar = wrap(after["yawY"] - before["yawY"])
        chord = math.dist(before["ar"], after["ar"])

        print(f"  wheels turned   {math.degrees(d_odom):+7.2f} deg")
        print(f"  ARCore turned   {math.degrees(d_ar):+7.2f} deg")
        if abs(d_odom) < math.radians(20):
            raise SystemExit("the robot barely turned; is it on the ground with motors on?")
        results["yaw_ratio"] = d_ar / d_odom
        # Rotating by theta about a centre at distance r moves the camera along a
        # chord of 2r*sin(theta/2). Invert it for the lever arm.
        results["lever_arm_m"] = chord / (2 * math.sin(abs(d_ar) / 2))

        # ---- drive forward -------------------------------------------------
        print(f"driving forward at {FORWARD_SPEED} m/s for {FORWARD_SECONDS}s ...")
        before = s.snap()
        s.pump(FORWARD_SECONDS, {"linearX": FORWARD_SPEED, "angular": 0.0})
        s.stop()
        s.pump(SETTLE)
        after = s.snap()
        require_tracking(after, "after driving")

        wheels = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        dx, dy, dz = (after["ar"][i] - before["ar"][i] for i in range(3))
        travelled = math.sqrt(dx * dx + dy * dy + dz * dz)
        horizontal = math.hypot(dx, dz)

        print(f"  wheels moved    {wheels:.3f} m")
        print(f"  ARCore moved    {travelled:.3f} m")
        if wheels < 0.05:
            raise SystemExit("the robot barely moved; is it on the ground with motors on?")
        results["scale"] = travelled / wheels
        results["vertical_drift_m"] = dy
        # Bearing about ARCore's up axis has the opposite sense to yaw_about_y,
        # so negate before comparing with the robot's heading.
        results["mount_yaw_deg"] = math.degrees(
            wrap(-math.atan2(dx, -dz) - before["yawY"]))
    finally:
        s.close()

    print()
    print("  ── calibration ──────────────────────────────────────────────")
    print(f"  scale, ARCore vs wheels : {results['scale']:.3f}"
          "        (1.0 = both metric and agreeing)")
    print(f"  yaw ratio               : {results['yaw_ratio']:+.3f}"
          "       (+1 same handedness, -1 flipped)")
    print(f"  mounting yaw            : {results['mount_yaw_deg']:+.1f} deg"
          "     (0 = phone faces robot-forward)")
    print(f"  lever arm               : {results['lever_arm_m']:.3f} m"
          "      (camera to turning centre)")
    print(f"  vertical drift          : {results['vertical_drift_m']:+.3f} m"
          "     (0 = phone level, ARCore y is gravity)")
    print()
    print("  A scale far from 1.0 means one estimator has the wrong units or the")
    print("  wheel diameter is misconfigured. A yaw ratio near -1 means the axes")
    print("  are handed oppositely and a sign is needed when relating them.")


if __name__ == "__main__":
    main()
