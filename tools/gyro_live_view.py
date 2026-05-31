#!/usr/bin/env python3
"""Realtime gyro Z viewer and bias estimator.

The MCU should print lines containing a named gz field, for example:
  GYRO,gz=0.123456,yaw=0.012,raw=4,ok=1
"""

from __future__ import annotations

import argparse
import math
import queue
import re
import statistics
import threading
import time
import tkinter as tk
from collections import deque

import serial


NAMED_GZ_RE = re.compile(
    r"(?:^|[,;\s])(?:g?z|gyro[_-]?z)\s*[:=,]\s*([-+]?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


def parse_gz(line: str) -> float | None:
    match = NAMED_GZ_RE.search(line)
    if match is None:
        return None
    return float(match.group(1))


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_queue: queue.Queue[tuple[float, float]]) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.error: str | None = None

    def run(self) -> None:
        try:
            with serial.Serial(self.port, self.baud, timeout=0.5) as ser:
                ser.reset_input_buffer()
                while not self.stop_event.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="ignore").strip()
                    gz = parse_gz(line)
                    if gz is not None:
                        self.out_queue.put((time.monotonic(), gz))
        except serial.SerialException as exc:
            self.error = str(exc)

    def stop(self) -> None:
        self.stop_event.set()


class GyroViewer:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.args = args
        self.queue: queue.Queue[tuple[float, float]] = queue.Queue()
        self.samples: deque[tuple[float, float]] = deque(maxlen=args.window)
        self.cal_samples: list[float] = []
        self.bias = 0.0
        self.bias_locked = False
        self.yaw = 0.0
        self.last_time: float | None = None

        self.reader = SerialReader(args.port, args.baud, self.queue)

        root.title(f"Gyro Z Bias Viewer - {args.port}")
        root.geometry("920x560")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status = tk.StringVar(value="Waiting for gz= samples...")
        self.stats = tk.StringVar(value="")
        self.canvas = tk.Canvas(root, bg="white", highlightthickness=1, highlightbackground="#c8c8c8")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=12, pady=(12, 6))

        info = tk.Frame(root)
        info.pack(fill=tk.X, padx=12, pady=(0, 8))

        tk.Label(info, textvariable=self.status, anchor="w").pack(fill=tk.X)
        tk.Label(info, textvariable=self.stats, anchor="w").pack(fill=tk.X)

        buttons = tk.Frame(root)
        buttons.pack(fill=tk.X, padx=12, pady=(0, 12))
        tk.Button(buttons, text="Reset Yaw", command=self.reset_yaw).pack(side=tk.LEFT)
        tk.Button(buttons, text="Recalibrate", command=self.recalibrate).pack(side=tk.LEFT, padx=8)
        tk.Button(buttons, text="Quit", command=self.on_close).pack(side=tk.RIGHT)

        self.reader.start()
        self.root.after(50, self.update)

    def reset_yaw(self) -> None:
        self.yaw = 0.0
        self.last_time = None

    def recalibrate(self) -> None:
        self.cal_samples.clear()
        self.bias = 0.0
        self.bias_locked = False
        self.reset_yaw()

    def on_close(self) -> None:
        self.reader.stop()
        self.root.destroy()

    def update(self) -> None:
        if self.reader.error:
            self.status.set(f"Serial error: {self.reader.error}")
            return

        got_sample = False
        while True:
            try:
                timestamp, gz = self.queue.get_nowait()
            except queue.Empty:
                break
            self.handle_sample(timestamp, gz)
            got_sample = True

        if got_sample:
            self.draw()
        self.root.after(50, self.update)

    def handle_sample(self, timestamp: float, gz: float) -> None:
        self.samples.append((timestamp, gz))

        if not self.bias_locked:
            self.cal_samples.append(gz)
            if len(self.cal_samples) >= self.args.samples:
                self.bias = statistics.fmean(self.cal_samples)
                self.bias_locked = True
                self.reset_yaw()
        else:
            if self.last_time is None:
                self.last_time = timestamp
            dt = timestamp - self.last_time
            self.last_time = timestamp
            if 0.0 < dt < 1.0:
                self.yaw += (gz - self.bias) * dt

    def draw(self) -> None:
        width = max(self.canvas.winfo_width(), 10)
        height = max(self.canvas.winfo_height(), 10)
        pad = 38
        self.canvas.delete("all")

        values = [sample[1] for sample in self.samples]
        if not values:
            return

        latest = values[-1]
        mean = statistics.fmean(values)
        stdev = statistics.pstdev(values) if len(values) > 1 else 0.0

        if self.bias_locked:
            corrected = latest - self.bias
            self.status.set(
                f"bias={self.bias:.6f} dps, gz={latest:.6f} dps, "
                f"corrected={corrected:.6f} dps, yaw={self.yaw:.3f} deg"
            )
        else:
            self.status.set(
                f"calibrating {len(self.cal_samples)}/{self.args.samples}, "
                f"current gz={latest:.6f} dps"
            )

        self.stats.set(
            f"visible mean={mean:.6f} dps, stdev={stdev:.6f} dps, "
            f"range={min(values):.6f} .. {max(values):.6f} dps"
        )

        center = self.bias if self.bias_locked else mean
        span = max(max(abs(v - center) for v in values), 0.05)
        span = max(span, self.args.yrange)
        y_min = center - span
        y_max = center + span

        plot_w = width - 2 * pad
        plot_h = height - 2 * pad
        self.canvas.create_rectangle(pad, pad, width - pad, height - pad, outline="#d0d0d0")

        zero_y = self.to_y(center, y_min, y_max, pad, plot_h)
        self.canvas.create_line(pad, zero_y, width - pad, zero_y, fill="#a0a0a0", dash=(4, 4))
        self.canvas.create_text(8, zero_y, anchor="w", text=f"{center:.3f}", fill="#606060")

        top_text = f"{y_max:.3f}"
        bottom_text = f"{y_min:.3f}"
        self.canvas.create_text(8, pad, anchor="w", text=top_text, fill="#606060")
        self.canvas.create_text(8, height - pad, anchor="w", text=bottom_text, fill="#606060")

        if len(values) < 2:
            return

        points: list[float] = []
        count = len(values)
        for index, value in enumerate(values):
            x = pad + (index / max(count - 1, 1)) * plot_w
            y = self.to_y(value, y_min, y_max, pad, plot_h)
            points.extend([x, y])

        self.canvas.create_line(points, fill="#0b62d6", width=2)
        if self.bias_locked and math.isfinite(self.bias):
            bias_y = self.to_y(self.bias, y_min, y_max, pad, plot_h)
            self.canvas.create_line(pad, bias_y, width - pad, bias_y, fill="#d62728")
            self.canvas.create_text(width - pad - 4, bias_y - 12, anchor="e", text="bias", fill="#d62728")

    @staticmethod
    def to_y(value: float, y_min: float, y_max: float, pad: int, plot_h: int) -> float:
        return pad + (y_max - value) / (y_max - y_min) * plot_h


def main() -> int:
    parser = argparse.ArgumentParser(description="Realtime gyro Z bias viewer.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("-n", "--samples", type=int, default=1500, help="Samples used to estimate bias.")
    parser.add_argument("--window", type=int, default=500, help="Number of visible samples.")
    parser.add_argument("--yrange", type=float, default=0.5, help="Minimum half range of Y axis in dps.")
    args = parser.parse_args()

    root = tk.Tk()
    GyroViewer(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
