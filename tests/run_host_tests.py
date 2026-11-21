"""Run the real task/vision/PID code on a host GCC with hardware stubs."""
from pathlib import Path
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "build" / "host-tests"
OUT.mkdir(parents=True, exist_ok=True)
INCLUDES = ["tests/stubs", "App_Task", "User_gx/Inc", "User_gx/App_Servo",
            "User_gx/App_Motor", "User_gx/App_LinePID", "User_gx/App_Protocol", "MyLib/Inc"]

def run(name, source, override=None):
    exe = OUT / (name + (".exe" if os.name == "nt" else ""))
    compiler = shutil.which(os.environ.get("CC", "gcc"))
    if compiler is None:
        raise RuntimeError("Host GCC not found; set CC to its executable path")
    env = os.environ.copy()
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
    cmd = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0"]
    cmd += ["-I" + str(ROOT / path) for path in INCLUDES]
    if override:
        cmd += ["-include", str(override)]
    cmd += [str(ROOT / source), str(ROOT / "User_gx/App_LinePID/app_line_pid.c"),
            str(ROOT / "MyLib/Src/pid.c"), "-lm", "-o", str(exe)]
    subprocess.run(cmd, cwd=ROOT, env=env, check=True)
    subprocess.run([str(exe)], cwd=ROOT, env=env, check=True)

run("zone", "tests/zone_control_test.c")
run("green", "tests/green_recovery_test.c")
run("zone_y_tuning", "tests/zone_y_tuning_test.c")
run("zone_xy_heading", "tests/zone_xy_heading_test.c")
override = OUT / "zone_override.h"
override.write_text('#include "app_task_config.h"\n' + "\n".join(
    f"#undef {name}\n#define {name} {value}" for name, value in {
        "APP_TASK_ZONE_AIM_X": "300.0f", "APP_TASK_ZONE_X_TOL": "6.0f",
        "APP_TASK_ZONE_AIM_Y": "360.0f", "APP_TASK_ZONE_Y_TOL": "15.0f",
    }.items()) + "\n", encoding="utf-8")
run("zone_independent_xy", "tests/zone_control_test.c", override)
run("green_with_zone_override", "tests/green_recovery_test.c", override)
run("zone_xy_heading_independent_xy", "tests/zone_xy_heading_test.c", override)
