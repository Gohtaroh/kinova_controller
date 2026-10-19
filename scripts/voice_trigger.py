#!/usr/bin/env python3
"""Voice trigger for the handover study: detect a spoken request, tell the controller.

Listens to the USB microphone, decides "is someone speaking?" with WebRTC VAD
(a speech/non-speech classifier, not a loudness threshold), and when speech
lasts long enough it sends "VOICE <level dBFS>" to KinovaController over
loopback UDP (127.0.0.1:47801). Every 0.1 s it also sends
"STAT <level> <noise floor> <speech fraction>" so the controller GUI can show
the live level. The LOUDNESS threshold is set in the controller GUI
("Voice min level [dBFS]"): the controller ignores VOICE messages quieter than
it. This script only decides "is it speech?".

NO AUDIO IS SAVED (ethics approval): samples are only analysed in memory.
The optional --log file contains timestamps and numbers only.

Run it in a terminal and leave it open; it prints a live status line and one
line per trigger, so it can also be used on its own to check for a long time
that robot / lab noise does not trigger it and that speech does:
    python3 ~/workspace/src/kinova_controller/scripts/voice_trigger.py
    python3 ~/workspace/src/kinova_controller/scripts/voice_trigger.py --no-send   # test only
Ctrl+C prints a summary and quits.

Decision rule (all tunable):
  * 30 ms frames, VAD aggressiveness --aggr (0-3, 3 = rejects the most noise)
  * a frame counts as speech only if VAD says so AND it is --margin-db louder
    than the running noise floor (rejects quiet speech-like hum)
  * trigger when >= --min-voiced of the frames in the last --window seconds are
    speech (default 0.30 s window, 80 %: a short clank or click cannot pass)
  * after a trigger, ignore further speech for --refractory seconds
"""
import argparse
import collections
import math
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


def find_device(name):
    for i, d in enumerate(sd.query_devices()):
        if d["max_input_channels"] > 0 and name.lower() in d["name"].lower():
            return i
    return None


def dbfs(x):
    rms = math.sqrt(float(np.mean(x.astype(np.float64) ** 2))) if len(x) else 0.0
    return 20.0 * math.log10(max(rms, 1e-9) / 32768.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="fifine", help="input device name substring (default: fifine)")
    ap.add_argument("--port", type=int, default=47801)
    ap.add_argument("--no-send", action="store_true", help="only print, do not notify the controller")
    ap.add_argument("--aggr", type=int, default=3, choices=range(4), help="VAD aggressiveness 0-3 (default 3)")
    ap.add_argument("--window", type=float, default=0.30, help="speech must fill this window [s] (default 0.30)")
    ap.add_argument("--min-voiced", type=float, default=0.80, help="fraction of speech frames in the window")
    ap.add_argument("--margin-db", type=float, default=8.0, help="speech must be this much above the noise floor")
    ap.add_argument("--refractory", type=float, default=1.0, help="ignore speech this long after a trigger [s]")
    ap.add_argument("--log", default="", help="append trigger timestamps/stats (no audio) to this text file")
    a = ap.parse_args()

    dev = find_device(a.device)
    if dev is None:
        sys.exit(f"input device containing '{a.device}' not found (see: python3 -m sounddevice)")
    vad = webrtcvad.Vad(a.aggr)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = ("127.0.0.1", a.port)
    nwin = max(1, int(round(a.window * 1000 / FRAME_MS)))
    win = collections.deque(maxlen=nwin)
    win_lvl = collections.deque(maxlen=nwin)
    last_stat = 0.0
    noise_db = None
    last_trigger = -1e9
    triggers = 0
    t_start = time.time()
    frames = 0
    speech_frames = 0
    peak_db = -120.0
    last_status = 0.0
    logf = open(a.log, "a") if a.log else None
    zero_frames = 0

    print(f"[voice] mic '{sd.query_devices(dev)['name']}' @ {RATE} Hz, VAD aggr {a.aggr}, "
          f"window {a.window:.2f}s >= {a.min_voiced:.0%}, margin {a.margin_db:.0f} dB, refractory {a.refractory:.1f}s, "
          f"{'NOT sending (test)' if a.no_send else f'sending VOICE to {addr[0]}:{addr[1]}'}. No audio is saved.",
          flush=True)

    with sd.InputStream(device=dev, samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME) as stream:
        try:
            while True:
                block, _ = stream.read(FRAME)
                x = block[:, 0]
                now = time.time()
                frames += 1
                lvl = dbfs(x)
                peak_db = max(peak_db, lvl)
                if not np.any(x):
                    zero_frames += 1
                # Noise floor: slow-tracking low percentile of the level.
                if noise_db is None:
                    noise_db = lvl
                elif lvl < noise_db:
                    noise_db = 0.9 * noise_db + 0.1 * lvl
                else:
                    noise_db = 0.999 * noise_db + 0.001 * lvl
                is_speech = vad.is_speech(x.tobytes(), RATE) and lvl > noise_db + a.margin_db
                speech_frames += is_speech
                win.append(is_speech)
                win_lvl.append(lvl)
                frac = sum(win) / len(win)
                if (len(win) == nwin and frac >= a.min_voiced and now - last_trigger >= a.refractory):
                    last_trigger = now
                    triggers += 1
                    voice_db = max(win_lvl)  # loudest frame of the speech window
                    stamp = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now * 1000) % 1000:03d}"
                    if not a.no_send:
                        for _ in range(3):
                            sock.sendto(f"VOICE {voice_db:.1f}".encode(), addr)
                    print(f"\n[voice] {stamp} TRIGGER #{triggers}  voice {voice_db:5.1f} dBFS  floor {noise_db:5.1f}"
                          f"{'' if a.no_send else '  -> VOICE sent'}", flush=True)
                    if logf:
                        logf.write(f"{stamp}\ttrigger\t{triggers}\tvoice_dbfs={voice_db:.1f}\tfloor_dbfs={noise_db:.1f}\n")
                        logf.flush()
                if not a.no_send and now - last_stat >= 0.1:
                    last_stat = now
                    sock.sendto(f"STAT {lvl:.1f} {noise_db:.1f} {frac:.2f}".encode(), addr)
                if now - last_status >= 0.5:
                    last_status = now
                    bar = "#" * int(max(0.0, min(30.0, (lvl + 70.0) / 2.0)))
                    hint = "  (NO SIGNAL: mic muted?)" if zero_frames == frames else ""
                    sys.stdout.write(f"\r[voice] {time.strftime('%H:%M:%S')} level {lvl:6.1f} dB  floor {noise_db:6.1f} "
                                     f" speech {frac:4.0%}  {bar:<30s} triggers {triggers}{hint}   ")
                    sys.stdout.flush()
        except KeyboardInterrupt:
            pass
    dur = time.time() - t_start
    print(f"\n[voice] summary: {dur / 60:.1f} min, {triggers} triggers, speech frames {speech_frames / max(frames, 1):.1%}, "
          f"peak level {peak_db:.1f} dBFS", flush=True)
    if logf:
        logf.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}\tsummary\tminutes={dur / 60:.1f}\ttriggers={triggers}\n")
        logf.close()


if __name__ == "__main__":
    main()
