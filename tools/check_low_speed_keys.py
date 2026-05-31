from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        print(f"FAIL: {message}")
        sys.exit(1)


def require_present(text: str, pattern: str, message: str) -> None:
    require(re.search(pattern, text, re.S) is not None, message)


def main() -> None:
    key_h = read_text("BSP/KEY/key.h")
    control_h = read_text("control.h")
    control_c = read_text("control.c")
    empty_c = read_text("empty.c")
    motor_h = read_text("BSP/MOTOR/motor.h")
    motor_c = read_text("BSP/MOTOR/motor.c")
    syscfg = read_text("empty.syscfg")

    require('pin.$assign = "PB24"' in syscfg, "PB24 should be configured in SysConfig")
    require('pin.$assign = "PB25"' in syscfg, "PB25 should be configured in SysConfig")

    for symbol in ("Key_Key4IsPressed", "Key_Key5IsPressed"):
        require(symbol in key_h, f"{symbol} should be declared in key.h")

    for macro in ("Key_Key4_IOMUX", "Key_Key5_IOMUX"):
        require(macro in key_h, f"{macro} pull-up should be enabled")

    require_present(
        key_h,
        r"Key_AllReleased\s*\([^)]*\).*?Key_Key4IsPressed\s*\(\)\s*==\s*0U.*?"
        r"Key_Key5IsPressed\s*\(\)\s*==\s*0U",
        "Key_AllReleased should include Key4 and Key5",
    )

    for selection in ("CONTROL_START_TASK2_LOW_SPEED", "CONTROL_START_TASK3_LOW_SPEED"):
        require(selection in control_h, f"{selection} should be in ControlStartSelection")
        require(selection in control_c, f"{selection} should be selected by control.c")
        require(selection in empty_c, f"{selection} should be handled by main")

    require_present(
        control_c,
        r"#define\s+CONTROL_DEFAULT_BASE_SPEED_MM_S\s+\(220\.0f\)",
        "default task base speed should remain 220 mm/s",
    )
    require_present(
        control_c,
        r"#define\s+CONTROL_LOW_BASE_SPEED_MM_S\s+\(150\.0f\)",
        "low-speed shortcut base speed should be 150 mm/s",
    )
    require_present(
        control_c,
        r"static\s+float\s+g_baseSpeedMmS\s*=\s*CONTROL_DEFAULT_BASE_SPEED_MM_S",
        "runtime base speed should be stored in g_baseSpeedMmS",
    )
    require_present(
        control_c,
        r"static\s+void\s+Control_SetTaskBaseSpeed\s*\(\s*float\s+baseSpeedMmS\s*\)",
        "task start should set the runtime base speed",
    )
    require_present(
        control_c,
        r"Control_UpdateLineTrack.*?float\s+baseSpeed\s*=\s*g_baseSpeedMmS",
        "line tracking should use the runtime base speed",
    )
    require_present(
        control_c,
        r"Control_StartTask2LowSpeed\s*\(\s*void\s*\).*?"
        r"Control_StartTask2WithBaseSpeed\s*\(\s*CONTROL_LOW_BASE_SPEED_MM_S\s*\)",
        "low-speed task2 wrapper should start task2 at 150 mm/s",
    )
    require_present(
        control_c,
        r"Control_StartTask3LowSpeedOneLap\s*\(\s*void\s*\).*?"
        r"Control_StartTask3WithBaseSpeed\s*\(\s*1U\s*,\s*CONTROL_LOW_BASE_SPEED_MM_S\s*\)",
        "low-speed task3 wrapper should start exactly one lap at 150 mm/s",
    )
    require_present(
        motor_h,
        r"void\s+SpeedLoop_SetRampStep\s*\(\s*float\s+rampStepMmS\s*\)",
        "speed loop should expose runtime ramp-step configuration",
    )
    require_present(
        motor_c,
        r"#define\s+SPEED_RAMP_STEP_DEFAULT_MM_S\s+\(10\.0f\)",
        "default speed ramp should remain 10 mm/s per 10 ms",
    )
    require_present(
        motor_c,
        r"static\s+volatile\s+float\s+s_rampStepMmS\s*=\s*SPEED_RAMP_STEP_DEFAULT_MM_S",
        "speed loop should store runtime ramp-step in s_rampStepMmS",
    )
    require_present(
        control_c,
        r"#define\s+CONTROL_LOW_RAMP_STEP_MM_S\s+\(5\.0f\)",
        "low-speed shortcut ramp step should be 5 mm/s per 10 ms",
    )
    require_present(
        control_c,
        r"Control_StartTask2WithBaseSpeed\s*\(.*?CONTROL_LOW_BASE_SPEED_MM_S.*?"
        r"SpeedLoop_SetRampStep\s*\(\s*CONTROL_LOW_RAMP_STEP_MM_S\s*\).*?"
        r"SpeedLoop_SetRampStep\s*\(\s*CONTROL_DEFAULT_RAMP_STEP_MM_S\s*\)",
        "low-speed task2 should use the low ramp step while non-low task2 keeps the default ramp step",
    )
    require_present(
        control_c,
        r"Control_StartTask3WithBaseSpeed\s*\(.*?CONTROL_LOW_BASE_SPEED_MM_S.*?"
        r"SpeedLoop_SetRampStep\s*\(\s*CONTROL_LOW_RAMP_STEP_MM_S\s*\).*?"
        r"SpeedLoop_SetRampStep\s*\(\s*CONTROL_DEFAULT_RAMP_STEP_MM_S\s*\)",
        "low-speed task3 should use the low ramp step while non-low task3 keeps the default ramp step",
    )
    require_present(
        control_c,
        r"CONTROL_STATE_TASK2_CENTER_TO_C",
        "low-speed task2 should have a center-before-track state for B-to-C",
    )
    require_present(
        control_c,
        r"CONTROL_STATE_TASK2_CENTER_TO_A",
        "low-speed task2 should have a center-before-track state for D-to-A",
    )
    require_present(
        control_c,
        r"CONTROL_STATE_TASK3_CENTER_TO_B",
        "low-speed task3 should have a center-before-track state for C-to-B",
    )
    require_present(
        control_c,
        r"CONTROL_STATE_TASK3_CENTER_TO_A",
        "low-speed task3 should have a center-before-track state for D-to-A",
    )
    require_present(
        control_c,
        r"#define\s+CONTROL_CENTER_LEFT_BIT\s+\(3U\).*?"
        r"#define\s+CONTROL_CENTER_RIGHT_BIT\s+\(4U\).*?"
        r"#define\s+CONTROL_CENTER_STABLE_TICKS\s+\(3U\)",
        "line-centering should require gray bits 3 and 4 to be stable for 3 ticks",
    )
    require_present(
        control_c,
        r"#define\s+CONTROL_CENTER_TURN_MIN_SPEED_MM_S\s+\(18\.0f\).*?"
        r"#define\s+CONTROL_CENTER_TURN_SPEED_LIMIT\s+\(35\.0f\)",
        "line-centering turn speed should be limited to 18..35 mm/s",
    )
    require_present(
        control_c,
        r"Control_UpdateLineCenter\s*\(.*?Control_CenterBitsOnBlack.*?"
        r"CONTROL_CENTER_STABLE_TICKS.*?motorA\.ref\s*=.*?motorB\.ref\s*=",
        "line-centering helper should wait for center bits and rotate in place",
    )
    require_present(
        control_c,
        r"CONTROL_STATE_TASK2_AB:.*?g_lowSpeedTaskActive.*?"
        r"CONTROL_STATE_TASK2_CENTER_TO_C.*?CONTROL_STATE_TASK2_TRACK_TO_C",
        "low-speed task2 should center after B before tracking to C",
    )
    require_present(
        control_c,
        r"CONTROL_STATE_TASK2_STRAIGHT_TO_D:.*?g_lowSpeedTaskActive.*?"
        r"CONTROL_STATE_TASK2_CENTER_TO_A.*?CONTROL_STATE_TASK2_TRACK_TO_A",
        "low-speed task2 should center after D before tracking to A",
    )
    require_present(
        control_c,
        r"CONTROL_STATE_TASK3_SEEK_C:.*?CONTROL_STATE_TASK3_CENTER_TO_B.*?"
        r"CONTROL_STATE_TASK3_TRACK_TO_B",
        "low-speed task3 should center after C before tracking to B",
    )
    require_present(
        control_c,
        r"CONTROL_STATE_TASK3_SEEK_D:.*?CONTROL_STATE_TASK3_CENTER_TO_A.*?"
        r"CONTROL_STATE_TASK3_TRACK_TO_A",
        "low-speed task3 should center after D before tracking to A",
    )
    require_present(
        control_c,
        r"Control_WaitForLowSpeedSelection\s*\(.*?Key_UserIsPressed\s*\(\)\s*!=\s*0U.*?"
        r"selection\s*=\s*CONTROL_START_TASK2_LOW_SPEED",
        "PB25 low-speed mode + Key_User should select low-speed task2",
    )
    require_present(
        control_c,
        r"Control_WaitForLowSpeedSelection\s*\(.*?Key_Key1IsPressed\s*\(\)\s*!=\s*0U.*?"
        r"selection\s*=\s*CONTROL_START_TASK3_LOW_SPEED",
        "PB25 low-speed mode + Key1 should select low-speed task3",
    )
    require_present(
        control_c,
        r"Key_Key5IsPressed\s*\(\)\s*!=\s*0U.*?"
        r"Control_WaitForLowSpeedSelection\s*\(",
        "PB25 should enter low-speed selection mode instead of directly starting a task",
    )
    require(
        re.search(
            r"else\s+if\s*\([^)]*Key_Key4IsPressed\s*\(\)\s*!=\s*0U.*?"
            r"selection\s*=\s*CONTROL_START_TASK2_LOW_SPEED",
            control_c,
            re.S,
        ) is None,
        "PB24 should no longer directly select low-speed task2",
    )
    require_present(
        empty_c,
        r"case\s+CONTROL_START_TASK3_LOW_SPEED\s*:.*?Control_StartTask3LowSpeedOneLap\s*\(",
        "main should start low-speed task3 as one low-speed lap",
    )
    require_present(
        empty_c,
        r"case\s+CONTROL_START_TASK2_LOW_SPEED\s*:.*?Control_StartTask2LowSpeed\s*\(",
        "main should start low-speed task2 at low speed",
    )

    print("PB25 low-speed selection checks passed")


if __name__ == "__main__":
    main()
