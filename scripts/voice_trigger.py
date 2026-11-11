#!/usr/bin/env python3
"""Microphone feed for the handover study's voice trigger.

Listens to the USB microphone and, every 0.1 s, sends to KinovaController
over loopback UDP (127.0.0.1:47801):
    STAT <level dBFS> <noise floor dBFS> <speech fraction 0-1> <peak dBFS> <all-zero 0/1>
"speech fraction" is the share of 30 ms frames in that 0.1 s that
  * WebRTC VAD (a speech/non-speech classifier, not a loudness threshold) judged
    to be speech,
  * were --margin-db above the running noise floor, and
  * were VOICED: periodic like vocal-fold vibration (normalized autocorrelation
    >= --voicing at a pitch of 80-400 Hz). Keyboard clicks, taps on the mic and
    clanks are not periodic, so they fail this test even when VAD is fooled.

ALL DECISIONS ARE MADE IN THE CONTROLLER (GUI "Bottle picks"): the minimum
voice level and the minimum speech duration (a tap on the mic or a clank is
far shorter than a spoken request) are set there. NO AUDIO IS SAVED: samples
are only analysed in memory.

KinovaController starts this script automatically with mc_kortex / mc_mujoco
(--quiet) and stops it on exit. To watch the mic on its own in a terminal:
    python3 ~/workspace/src/kinova_controller/scripts/voice_trigger.py
It prints a live status line and marks speech that lasted --show-duration
seconds (display only). Ctrl+C prints a summary. Only one program can use the
mic at a time: stop the controller first (or this script) if the mic is busy.
"""
import argparse
import math
import os
import signal
import socket
import sys
import time

import numpy as np

try:
    import sounddevice as sd
    import webrtcvad
except ImportError as e:  # pragma: no cover
    sys.exit(f"missing module: {e.name} (pip3 install --user sounddevice webrtcvad-wheels)")

RATE = 48000  # the fifine mic accepts 44.1/48 kHz; WebRTC VAD supports 48 kHz
FRAME_MS = 30
FRAME = RATE * FRAME_MS // 1000
STAT_FRAMES = 3  # 3 x 30 ms ~ every 0.1 s


def find_device(name):
    for i, d in enumerate(sd.query_devices()):
        if d["max_input_channels"] > 0 and name.lower() in d["name"].lower():
            return i
    return None


def voicing(x):
    """Peak normalized autocorrelation in the 80-400 Hz pitch range (0..1)."""
    y = x[::3].astype(np.float64)  # 48 kHz -> 16 kHz is plenty for pitch
    y = y - y.mean()
    e = float(np.dot(y, y))
    if e <= 0.0:
        return 0.0
    n = len(y)
    best = 0.0
    for lag in range(16000 // 400, min(16000 // 80, n - 1) + 1):  # 40..200 samples
        a, b = y[:-lag], y[lag:]
        den = math.sqrt(float(np.dot(a, a)) * float(np.dot(b, b)))
        if den > 0.0:
            best = max(best, float(np.dot(a, b)) / den)
    return best


def dbfs(x):
    rms = math.sqrt(float(np.mean(x.astype(np.float64) ** 2))) if len(x) else 0.0
    return 20.0 * math.log10(max(rms, 1e-9) / 32768.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="fifine", help="input device name substring (default: fifine)")
    ap.add_argument("--port", type=int, default=47801)
    ap.add_argument("--aggr", type=int, default=3, choices=range(4), help="VAD aggressiveness 0-3 (default 3)")
    ap.add_argument("--margin-db", type=float, default=8.0, help="speech must be this much above the noise floor")
    ap.add_argument("--voicing", type=float, default=0.5,
                    help="min periodicity (0-1) of a frame to count as voiced speech (default 0.5)")
    ap.add_argument("--show-duration", type=float, default=1.0,
                    help="terminal display only: mark speech that lasted this long [s] (controller decides)")
    ap.add_argument("--quiet", action="store_true", help="no live status line (used when the controller starts it)")
    a = ap.parse_args()

    # Stopped by the controller with SIGTERM: leave at once. (2026-11-09: raising
    # KeyboardInterrupt from the handler left a watcher alive after mc_kortex quit,
    # holding the mic so the next mc_kortex could not start its own.)
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
    dev = find_device(a.device)
    if dev is None:
        sys.exit(f"[voice] input device containing '{a.device}' not found (is the mic plugged in?)")
    vad = webrtcvad.Vad(a.aggr)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = ("127.0.0.1", a.port)
    noise_db = None
    t_start = time.time()
    frames = zero_frames = speech_frames = 0
    blk_speech = blk_n = 0
    blk_zero = 0  # frames in this block whose samples are all exactly zero (muted / no signal)
    blk_peak = -120.0
    run_s = gap_s = 0.0  # display-only utterance tracking
    marked = 0
    last_status = 0.0
    peak_db = -120.0

    print(f"[voice] mic '{sd.query_devices(dev)['name']}' @ {RATE} Hz, VAD aggr {a.aggr}, margin {a.margin_db:.0f} dB, "
          f"voicing >= {a.voicing:.2f}, "
          f"sending levels to {addr[0]}:{addr[1]} every 0.1 s. No audio is saved.", flush=True)
    try:
        with sd.InputStream(device=dev, samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME) as stream:
            while True:
                block, _ = stream.read(FRAME)
                x = block[:, 0]
                frames += 1
                lvl = dbfs(x)
                peak_db = max(peak_db, lvl)
                if not np.any(x):
                    zero_frames += 1
                    blk_zero += 1
                if noise_db is None:
                    noise_db = lvl
                elif lvl < noise_db:
                    noise_db = 0.9 * noise_db + 0.1 * lvl
                else:
                    noise_db = 0.999 * noise_db + 0.001 * lvl
                is_speech = (lvl > noise_db + a.margin_db and vad.is_speech(x.tobytes(), RATE)
                             and voicing(x) >= a.voicing)
                speech_frames += is_speech
                blk_speech += is_speech
                blk_n += 1
                blk_peak = max(blk_peak, lvl)
                if blk_n < STAT_FRAMES:
                    continue
                frac = blk_speech / blk_n
                try:
                    # 5th field (2026-11-09): 1 = every sample of this block was exactly zero,
                    # i.e. the mic is muted or delivers no signal (the controller shows a warning).
                    sock.sendto(f"STAT {lvl:.1f} {noise_db:.1f} {frac:.2f} {blk_peak:.1f} "
                                f"{1 if blk_zero == blk_n else 0}".encode(), addr)
                except OSError:
                    pass
                # Display-only utterance tracking (same idea as the controller).
                dt = blk_n * FRAME_MS / 1000.0
                if frac >= 0.5:
                    run_s += dt
                    gap_s = 0.0
                else:
                    gap_s += dt
                    if gap_s > 0.4:
                        if 0.0 < run_s < a.show_duration and not a.quiet:
                            print(f"\n[voice] {time.strftime('%H:%M:%S')} short sound {run_s:.1f} s (too short)",
                                  flush=True)
                        run_s = 0.0
                if run_s >= a.show_duration and run_s - dt < a.show_duration:
                    marked += 1
                    if not a.quiet:
                        print(f"\n[voice] {time.strftime('%H:%M:%S')} speech >= {a.show_duration:.1f} s (#{marked}), "
                              f"peak {blk_peak:.1f} dBFS", flush=True)
                blk_speech = blk_n = 0
                blk_zero = 0
                blk_peak = -120.0
                now = time.time()
                if not a.quiet and now - last_status >= 0.5:
                    last_status = now
                    bar = "#" * int(max(0.0, min(30.0, (lvl + 70.0) / 2.0)))
                    hint = "  (NO SIGNAL: mic muted?)" if zero_frames == frames else ""
                    sys.stdout.write(f"\r[voice] {time.strftime('%H:%M:%S')} level {lvl:6.1f} dB  floor {noise_db:6.1f} "
                                     f" speech {frac:4.0%}  {bar:<30s}{hint}   ")
                    sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    dur = time.time() - t_start
    if not a.quiet:
        print(f"\n[voice] summary: {dur / 60:.1f} min, {marked} speech >= {a.show_duration:.1f} s, "
              f"speech frames {speech_frames / max(frames, 1):.1%}, peak {peak_db:.1f} dBFS", flush=True)


if __name__ == "__main__":
    main()
