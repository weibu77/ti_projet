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


def require_absent(text: str, pattern: str, message: str) -> None:
    require(re.search(pattern, text) is None, message)


def require_present(text: str, pattern: str, message: str) -> None:
    require(re.search(pattern, text) is not None, message)


def main() -> None:
    control = read_text("control.c")
    uart = read_text("BSP/UART/uart.c")
    gray_c = read_text("BSP/GRAY/gray_sensor.c")
    gray_h = read_text("BSP/GRAY/gray_sensor.h")
    readme = read_text("README.md")

    for old_name in (
        "CONTROL_ENCODER_DISPLAY_MAX_ABS",
        "Control_ResetEncoderAndDisplay",
        "Control_PrimeEncoderDisplay",
        "Control_OLEDShowEncoderPins",
        "Control_OLEDShowEncoderDebug",
    ):
        require(old_name not in control, f"{old_name} should be removed from control.c")

    require_absent(uart, r'StrEqual\(op,\s*"M"\)', "UART M open-loop command should be removed")
    require_absent(uart, r'"M_FORMAT"', "UART M_FORMAT error should be removed")
    require_absent(readme, r'M left right', "README should not document UART M command")
    require_absent(readme, r'编码器诊断|check_encoder_oled_debug|check_motor_encoder_diagnostics',
                   "README should not document removed encoder diagnostics")

    for script in (
        "tools/check_encoder_oled_debug.py",
        "tools/check_encoder_direction.py",
        "tools/check_motor_encoder_diagnostics.py",
    ):
        require(not (ROOT / script).exists(), f"{script} should be removed")

    for state in (
        "CONTROL_GRAY_CAL_IDLE",
        "CONTROL_GRAY_CAL_WAIT_WHITE",
        "CONTROL_GRAY_CAL_WAIT_BLACK",
        "CONTROL_GRAY_CAL_READY_SAVE",
        "CONTROL_GRAY_CAL_SAVE_ERROR",
        "CONTROL_GRAY_CAL_DONE",
    ):
        require(state in control, f"{state} should exist in gray calibration state machine")

    for text in ("GRAY CAL", "MODE:ON", "WHITE OK", "BLACK OK", "SAVE OK"):
        require(text in control, f"OLED calibration text '{text}' should exist")

    require_present(control, r"GraySensor_SaveCalibration\s*\(", "Gray calibration flow should save to flash")
    require_present(control, r"GraySensor_Init\s*\(&g_gray_sensor,\s*g_grayCalWhite,\s*g_grayCalBlack\s*\)",
                    "Gray calibration save step should update active thresholds")
    require_present(control, r"Control_GrayCalibrationBlocksStart\s*\(",
                    "Task selection should be blocked while calibration is incomplete")
    require("GRAY_CAL,SAVED" in control, "UART should report flash save success")
    require(
        re.search(
            r"Control_OnlyGrayCalKeyPressed\s*\(.*?"
            r"Key_Key4IsPressed\s*\(\)\s*!=\s*0U",
            control,
            re.S,
        ) is not None,
        "Key4 should be the gray calibration key",
    )
    require(
        re.search(r"Control_OnlyGrayCalKeyPressed\s*\(.*?Key_Key3IsPressed\s*\(\)\s*!=\s*0U", control, re.S) is None,
        "Key3 should no longer be the gray calibration key",
    )

    for symbol in (
        "GRAY_SENSOR_CAL_FLASH_ADDRESS",
        "GRAY_SENSOR_CAL_FLASH_SIZE",
        "GraySensor_LoadSavedCalibration",
        "GraySensor_SaveCalibration",
    ):
        require(symbol in gray_h, f"{symbol} should be declared in gray_sensor.h")
        require(symbol in gray_c, f"{symbol} should be implemented in gray_sensor.c")

    require("0x0001FC00" in gray_h or "0x0001FC00" in gray_c,
            "Gray calibration storage should use the last 1 KB flash sector")
    require("DL_FlashCTL_eraseMemoryFromRAM" in gray_c,
            "Flash erase should use the DriverLib RAM flash routine")
    require("DL_FlashCTL_programMemoryFromRAM64" in gray_c,
            "Flash programming should use the DriverLib RAM flash routine")

    map_path = ROOT / "Debug" / "empty_LP_MSPM0G3507_nortos_ticlang.map"
    if map_path.exists():
        map_text = map_path.read_text(encoding="utf-8", errors="ignore")
        match = re.search(r"FLASH\s+([0-9a-fA-F]{8})\s+[0-9a-fA-F]{8}\s+([0-9a-fA-F]{8})", map_text)
        require(match is not None, "Could not find FLASH usage in map file")
        used_end = int(match.group(1), 16) + int(match.group(2), 16)
        require(used_end <= 0x0001FC00,
                f"Linked image reaches 0x{used_end:08X}, overlapping gray calibration flash sector")

    print("Key3 flash calibration checks passed")


if __name__ == "__main__":
    main()
