#!/usr/bin/env python3
"""Mouse middle-click -> KinovaController software emergency stop.

Watches X11 raw mouse events with `xinput test-xi2 --root` (works whatever
window has focus, no root needed) and, on a middle-button press, sends "ESTOP"
to the controller over loopback UDP (127.0.0.1:47800). It also sends "ALIVE"
every 0.5 s so the controller GUI ("Emergency stop" -> "Mouse middle-click
stop") shows ACTIVE while this script runs. Wheel scrolling (buttons 4-7) and
other buttons are ignored.

KinovaController starts this script automatically when the controller starts
(mc_kortex / mc_mujoco) and stops it on exit, so normally you never run it by
hand. Manual run (e.g. to test the click detection without a controller):
    python3 ~/workspace/src/kinova_controller/scripts/estop_mouse.py
Ctrl+C stops it.
"""
import argparse
import ctypes
import re
import signal
import socket
import subprocess
import sys
import threading
import time

MIDDLE = 2


def _die_with_parent():
    # Runs in the xinput child: get SIGTERM if this script dies (no orphans).
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG = 1
    except OSError:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=47800)
    ap.add_argument("--button", type=int, default=MIDDLE, help="X button number (2 = middle)")
    a = ap.parse_args()

    addr = ("127.0.0.1", a.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop = threading.Event()

    def heartbeat():
        while not stop.is_set():
            try:
                sock.sendto(b"ALIVE", addr)
            except OSError:
                pass
            stop.wait(0.5)

    threading.Thread(target=heartbeat, daemon=True).start()

    try:
        proc = subprocess.Popen(["xinput", "test-xi2", "--root"], stdout=subprocess.PIPE, text=True, bufsize=1,
                                preexec_fn=_die_with_parent)
    except FileNotFoundError:
        sys.exit("xinput not found")

    # Stopped by the controller with SIGTERM: leave through the finally block.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    print(f"[estop_mouse] watching mouse button {a.button}; sending ESTOP to {addr[0]}:{addr[1]} (Ctrl+C to quit)",
          flush=True)
    in_press = False
    last_fire = 0.0
    try:
        for line in proc.stdout:
            if line.startswith("EVENT type"):
                in_press = "(RawButtonPress)" in line
                continue
            if not in_press:
                continue
            m = re.match(r"\s+detail:\s+(\d+)", line)
            if not m:
                continue
            in_press = False
            if int(m.group(1)) != a.button:
                continue
            now = time.monotonic()
            if now - last_fire < 0.3:  # the same press is reported by master and slave devices
                continue
            last_fire = now
            for _ in range(3):  # UDP on loopback is reliable in practice; repeat anyway
                sock.sendto(b"ESTOP", addr)
            print(f"[estop_mouse] {time.strftime('%H:%M:%S')} middle click -> ESTOP sent", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        proc.terminate()
    if proc.poll() not in (None, 0, -15):
        sys.exit(f"xinput exited with {proc.returncode} (is an X11 display available? DISPLAY unset?)")


if __name__ == "__main__":
    main()
