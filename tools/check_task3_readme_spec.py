from pathlib import Path
import re
import sys


PROJECT_DIR = Path(__file__).resolve().parents[1]
CONTROL_C = PROJECT_DIR / "control.c"


def require(pattern: str, description: str, text: str) -> None:
    if re.search(pattern, text, re.S) is None:
        print(f"FAIL: {description}")
        sys.exit(1)


def main() -> None:
    text = CONTROL_C.read_text(encoding="utf-8")

    required_patterns = [
        (
            r"#define\s+CONTROL_TASK3_AC_FIRST_YAW\s+\(-44\.0f\)",
            "task3 A-C first-lap yaw is -44 deg",
        ),
        (
            r"#define\s+CONTROL_TASK3_AC_SECOND_YAW\s+\(-44\.0f\)",
            "task3 A-C second-lap yaw is -44 deg",
        ),
        (
            r"#define\s+CONTROL_TASK3_AC_THIRD_YAW\s+\(-44\.0f\)",
            "task3 A-C third-lap yaw is -44 deg",
        ),
        (
            r"#define\s+CONTROL_TASK3_BD_FIRST_YAW\s+\(225\.0f\)",
            "task3 B-D first-lap yaw is 225 deg",
        ),
        (
            r"#define\s+CONTROL_TASK3_BD_SECOND_YAW\s+\(225\.0f\)",
            "task3 B-D second-lap yaw is 225 deg",
        ),
        (
            r"#define\s+CONTROL_TASK3_BD_THIRD_YAW\s+\(225\.0f\)",
            "task3 B-D third-lap yaw is 225 deg",
        ),
        (
            r"#define\s+CONTROL_TASK3_MAX_LAPS\s+\(4U\)",
            "task3 max selectable lap count is 4",
        ),
        (
            r"if\s*\(\s*g_task3LapTarget\s*>\s*CONTROL_TASK3_MAX_LAPS\s*\).*?"
            r"g_task3LapTarget\s*=\s*CONTROL_TASK3_MIN_LAPS",
            "task3 key2 setup wraps 1..4 laps",
        ),
        (
            r"lapCount\s*>\s*CONTROL_TASK3_MAX_LAPS.*?"
            r"lapCount\s*=\s*CONTROL_TASK3_MAX_LAPS",
            "task3 explicit lap count clamps to max 4",
        ),
        (
            r"#define\s+CONTROL_TASK3_AC_DISTANCE_MM\s+\(1050U\)",
            "task3 A-C encoder distance is 1050 mm",
        ),
        (
            r"#define\s+CONTROL_TASK3_BD_DISTANCE_MM\s+\(1100U\)",
            "task3 B-D encoder distance is 1100 mm",
        ),
        (
            r"#define\s+CONTROL_TASK3_TURN_MIN_SPEED_MM_S\s+\(25\.0f\)",
            "task3 turn minimum speed is 25 mm/s",
        ),
        (
            r"#define\s+CONTROL_TASK3_TURN_SPEED_LIMIT\s+\(40\.0f\)",
            "task3 turn speed limit is 40 mm/s",
        ),
        (
            r"static\s+float\s+Control_Task3AcTargetYaw\s*\(void\)",
            "task3 selects A-C yaw by current lap",
        ),
        (
            r"static\s+float\s+Control_Task3BdTargetYaw\s*\(void\)",
            "task3 selects B-D yaw by current lap",
        ),
        (
            r"Control_Task3DistanceTargetCounts\s*\(\s*uint32_t\s+distanceMm\s*\)",
            "task3 distance helper accepts per-segment distance",
        ),
        (
            r"Control_StartTask3Turn\s*\(\s*Control_Task3AcTargetYaw\s*\(\s*\)\s*\)",
            "task3 starts and repeats A-C turn with per-lap yaw",
        ),
        (
            r"Control_StartTask3Drive\s*\(\s*Control_Task3AcTargetYaw\s*\(\s*\)\s*\)",
            "task3 A-C drive uses per-lap yaw target",
        ),
        (
            r"Control_Task3DistanceTargetCounts\s*\(\s*CONTROL_TASK3_AC_DISTANCE_MM\s*\)",
            "task3 A-C drive uses 1050 mm target",
        ),
        (
            r"Control_StartTask3Turn\s*\(\s*Control_Task3BdTargetYaw\s*\(\s*\)\s*\)",
            "task3 B-D turn uses per-lap yaw target",
        ),
        (
            r"Control_StartTask3Drive\s*\(\s*Control_Task3BdTargetYaw\s*\(\s*\)\s*\)",
            "task3 B-D drive uses per-lap yaw target",
        ),
        (
            r"Control_Task3DistanceTargetCounts\s*\(\s*CONTROL_TASK3_BD_DISTANCE_MM\s*\)",
            "task3 B-D drive uses 1100 mm target",
        ),
    ]

    for pattern, description in required_patterns:
        require(pattern, description, text)

    print("PASS: task3 README spec matches control.c")


if __name__ == "__main__":
    main()
