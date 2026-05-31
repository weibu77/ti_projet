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
    control = read_text("control.c")
    syscfg = read_text("empty.syscfg")
    readme = read_text("README.md")

    require_present(
        control,
        r"if\s*\(\s*on\s*!=\s*0U\s*\)\s*\{.*?"
        r"DL_GPIO_clearPins\s*\(\s*Alert_PORT\s*,\s*Alert_LED_PIN\s*\).*?"
        r"DL_GPIO_setPins\s*\(\s*Alert_PORT\s*,\s*Alert_BUZZER_PIN\s*\)",
        "Alert on should drive low-active LED low and high-active buzzer high",
    )
    require_present(
        control,
        r"\}\s*else\s*\{.*?"
        r"DL_GPIO_setPins\s*\(\s*Alert_PORT\s*,\s*Alert_LED_PIN\s*\).*?"
        r"DL_GPIO_clearPins\s*\(\s*Alert_PORT\s*,\s*Alert_BUZZER_PIN\s*\)",
        "Alert off should drive low-active LED high and high-active buzzer low",
    )
    require_present(
        control,
        r"Control_Init\s*\(.*?Control_AlertSet\s*\(\s*0U\s*\)",
        "Control_Init should leave the alert outputs off",
    )
    require(
        'GPIO7.associatedPins[0].initialValue = "SET";' in syscfg,
        "SysConfig should default Alert.LED high so low-active LED starts off",
    )
    require(
        "LED 低电平点亮，蜂鸣器高电平响" in readme,
        "README should document the mixed LED/buzzer polarities",
    )

    print("Alert polarity checks passed")


if __name__ == "__main__":
    main()
