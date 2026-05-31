#!/usr/bin/env python3
"""Collect gyro Z samples from serial and estimate zero-rate bias.

Input examples accepted by default:
  gz=0.123
  GZ,-12.4
  0.01,0.02,-0.15

For CSV input, select the zero-based column with --column.
Use --scale when the MCU prints raw ADC/IMU counts instead of dps.
For ICM42688 at +/-1000 dps, raw gyro sensitivity is typically 32.8 LSB/dps,
so use --scale 32.8 to convert raw gz to dps.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import time
from pathlib import Path

import serial


DEFAULT_NAMED_RE = re.compile(
    r"(?:^|[,;\s])(?:g?z|gyro[_-]?z)\s*[:=,]\s*([-+]?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)
NUMBER_RE = re.compile(r"[-+]?\d+(?:\.\d+)?")


def make_named_re(field: str) -> re.Pattern[str]:
    if field.lower() == "gz":
        return DEFAULT_NAMED_RE

    return re.compile(
        rf"(?:^|[,;\s]){re.escape(field)}\s*[:=,]\s*([-+]?\d+(?:\.\d+)?)",
        re.IGNORECASE,
    )


def parse_sample(line: str, column: int, require_named: bool, named_re: re.Pattern[str]) -> float | None:
    match = named_re.search(line)
    if match is not None:
        return float(match.group(1))

    if require_named:
        return None

    numbers = NUMBER_RE.findall(line)
    if len(numbers) <= column:
        return None

    return float(numbers[column])


def read_sample(
    port: serial.Serial,
    column: int,
    scale: float,
    require_named: bool,
    named_re: re.Pattern[str],
) -> float | None:
    raw = port.readline()
    if not raw:
        return None

    line = raw.decode("utf-8", errors="ignore").strip()
    value = parse_sample(line, column, require_named, named_re)
    if value is None:
        return None

    return value / scale


def collect_samples(
    port: serial.Serial,
    sample_count: int,
    discard_count: int,
    column: int,
    scale: float,
    require_named: bool,
    named_re: re.Pattern[str],
) -> list[float]:
    samples: list[float] = []
    discarded = 0

    print(f"Keep the car still. Discarding {discard_count} samples...")
    while discarded < discard_count:
        if read_sample(port, column, scale, require_named, named_re) is not None:
            discarded += 1

    print(f"Collecting {sample_count} gyro Z samples...")
    while len(samples) < sample_count:
        value = read_sample(port, column, scale, require_named, named_re)
        if value is None:
            continue
        samples.append(value)
        if len(samples) % 100 == 0 or len(samples) == sample_count:
            print(f"\r{len(samples)}/{sample_count}", end="", flush=True)

    print()
    return samples


def send_precommand(port: serial.Serial, command: str, delay_s: float) -> None:
    text = command.strip()
    if not text:
        return

    packet = f"{text}\r\n".encode("ascii")
    port.write(packet)
    port.flush()
    print(f"precmd      = {text}")

    if delay_s > 0.0:
        time.sleep(delay_s)

    while port.in_waiting:
        response = port.readline().decode("utf-8", errors="ignore").strip()
        if response:
            print(f"mcu         = {response}")


def run_monitor(
    port: serial.Serial,
    bias: float,
    column: int,
    scale: float,
    require_named: bool,
    named_re: re.Pattern[str],
) -> None:
    yaw = 0.0
    last_time = time.monotonic()

    print("Monitoring corrected gyro Z. Press Ctrl+C to stop.")
    while True:
        value = read_sample(port, column, scale, require_named, named_re)
        if value is None:
            continue

        now = time.monotonic()
        dt = now - last_time
        last_time = now

        corrected = value - bias
        yaw += corrected * dt
        print(f"gz={value: .6f} dps, bias={bias: .6f}, corrected={corrected: .6f}, yaw={yaw: .3f} deg")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calibrate gyro Z zero-rate bias from serial text output.",
    )
    parser.add_argument("--port", required=True, help="Serial port, for example COM8.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument(
        "--open-delay",
        type=float,
        default=0.2,
        help="Seconds to wait after opening the serial port before sending --precmd or collecting samples.",
    )
    parser.add_argument("-n", "--samples", type=int, default=1500, help="Number of valid samples.")
    parser.add_argument("--discard", type=int, default=100, help="Initial valid samples to ignore.")
    parser.add_argument(
        "--column",
        type=int,
        default=0,
        help="Zero-based numeric column used when the line is CSV without a gz= field.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Divide received value by this scale. Use 32.8 if printing raw ICM42688 gz at +/-1000 dps.",
    )
    parser.add_argument(
        "--output",
        default="gyro_z_bias.json",
        help="Path to save calibration result.",
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="After calibration, print corrected gz and integrated yaw.",
    )
    parser.add_argument(
        "--send-to-mcu",
        action="store_true",
        help="After calibration, send 'G <bias>' to the MCU so firmware uses this gyro Z bias.",
    )
    parser.add_argument(
        "--require-gz",
        action="store_true",
        help="Only accept lines with a named gz= or gyro_z= field. This ignores interleaved VOFA CSV lines.",
    )
    parser.add_argument(
        "--field",
        default="gz",
        help="Named field to read from serial text. Use gzc to average the post-quick-calibration residual.",
    )
    parser.add_argument(
        "--send-mode",
        choices=("absolute", "add"),
        default="absolute",
        help="With --send-to-mcu, send G <bias> for absolute bias or GA <bias> to add a residual to current bias.",
    )
    parser.add_argument(
        "--precmd",
        default="",
        help='Command sent to the MCU before collecting samples, for example "YM 0".',
    )
    parser.add_argument(
        "--precmd-delay",
        type=float,
        default=0.3,
        help="Seconds to wait after --precmd before collecting samples.",
    )
    args = parser.parse_args()

    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.discard < 0:
        raise SystemExit("--discard cannot be negative")
    if args.column < 0:
        raise SystemExit("--column cannot be negative")
    if args.scale == 0:
        raise SystemExit("--scale cannot be zero")
    if args.open_delay < 0.0:
        raise SystemExit("--open-delay cannot be negative")
    if args.precmd_delay < 0.0:
        raise SystemExit("--precmd-delay cannot be negative")

    named_re = make_named_re(args.field)

    with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
        if args.open_delay > 0.0:
            time.sleep(args.open_delay)
        ser.reset_input_buffer()
        send_precommand(ser, args.precmd, args.precmd_delay)

        samples = collect_samples(
            ser,
            args.samples,
            args.discard,
            args.column,
            args.scale,
            args.require_gz,
            named_re,
        )
        bias = statistics.fmean(samples)
        stdev = statistics.pstdev(samples) if len(samples) > 1 else 0.0

        result = {
            "port": args.port,
            "baud": args.baud,
            "samples": len(samples),
            "discard": args.discard,
            "column": args.column,
            "scale": args.scale,
            "require_gz": args.require_gz,
            "field": args.field,
            "send_mode": args.send_mode,
            "open_delay_s": args.open_delay,
            "precmd": args.precmd,
            "precmd_delay_s": args.precmd_delay,
            "bias_dps": bias,
            "stdev_dps": stdev,
            "min_dps": min(samples),
            "max_dps": max(samples),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }

        output_path = Path(args.output)
        output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

        print(f"gyro_z_bias = {bias:.8f} dps")
        print(f"stdev       = {stdev:.8f} dps")
        print(f"range       = {result['min_dps']:.8f} .. {result['max_dps']:.8f} dps")
        print(f"saved       = {output_path}")

        if args.send_to_mcu:
            op = "GA" if args.send_mode == "add" else "G"
            command = f"{op} {bias:.8f}\r\n".encode("ascii")
            ser.write(command)
            ser.flush()
            print(f"sent        = {command.decode('ascii').strip()}")
            time.sleep(0.2)
            while ser.in_waiting:
                response = ser.readline().decode("utf-8", errors="ignore").strip()
                if response:
                    print(f"mcu         = {response}")

        if args.monitor:
            run_monitor(ser, bias, args.column, args.scale, args.require_gz, named_re)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
