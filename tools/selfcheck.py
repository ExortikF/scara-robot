"""Проверка проекта одной командой: тесты, импорты, точки входа.

    python tools/selfcheck.py

Железо не нужно. Проверяется то, что ломается чаще всего при правках:

* доктесты всех модулей — в них живут проверенные числа и инварианты;
* каждый скрипт компилируется и отвечает на --help, то есть импорты
  целы и разбор аргументов не сломан;
* контур слежения доезжает до цели на симуляторе привода.

Последнее важнее прочего: именно этот путь ломался за время работы чаще
всего, и каждый раз это выглядело как «программа работает, а рука стоит».
"""

from __future__ import annotations

import doctest
import importlib
import logging
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

MODULES = ["registers", "driver", "follow", "armtrack", "goto", "discovery",
           "bus", "pulses", "kinematics", "teleop", "stepper_link",
           "simulation"]

GREEN, RED, DIM, OFF = "\033[32m", "\033[31m", "\033[2m", "\033[0m"


def line(ok: bool, text: str, detail: str = "") -> bool:
    mark = f"{GREEN}ок{OFF}" if ok else f"{RED}СБОЙ{OFF}"
    print(f"  {mark:<12} {text}" + (f"  {DIM}{detail}{OFF}" if detail else ""))
    return ok


def check_doctests() -> bool:
    print("доктесты:")
    logging.disable(logging.WARNING)
    good = True
    total = 0
    for name in MODULES:
        try:
            module = importlib.import_module(f"scara_drive.{name}")
        except Exception as exc:
            good = line(False, name, f"не импортируется: {exc}") and good
            continue
        result = doctest.testmod(module)
        total += result.attempted
        if result.failed:
            good = line(False, name, f"{result.failed} падений") and good
    logging.disable(logging.NOTSET)
    line(good, f"все модули, {total} проверок")
    return good


def check_entry_points() -> bool:
    print("\nточки входа:")
    good = True
    scripts = sorted(list((ROOT / "tools").glob("*.py"))
                     + list((ROOT / "examples").glob("*.py")))
    for script in scripts:
        if script.name == "selfcheck.py":
            continue
        done = subprocess.run([sys.executable, str(script), "--help"],
                              capture_output=True, timeout=60)
        if done.returncode != 0:
            tail = done.stderr.decode(errors="replace").strip().splitlines()
            good = line(False, script.name,
                        tail[-1] if tail else "ненулевой код возврата") and good
    line(good, f"{len(scripts) - 1} скриптов отвечают на --help")
    return good


def check_control_loop() -> bool:
    """Контур должен доезжать до цели на симуляторе привода."""
    print("\nконтур слежения:")
    from scara_drive.follow import AngleFollower, FollowConfig, MultiAxisFollower
    from scara_drive.simulation import SimulatedAxis

    cfg = FollowConfig(circular=False, invert_encoder=True, kp=0.6,
                       max_rpm=60, tolerance_deg=2.0, period_s=0.03,
                       min_motor_deg=-900, max_motor_deg=900)
    axis = AngleFollower(SimulatedAxis(latency_s=0.001), cfg)
    loop = MultiAxisFollower({"J": axis}, period_s=0.03)
    loop.set_zero_here()
    loop.start()

    try:
        # 1. Пока движение не разрешено, ось обязана стоять.
        axis.set_target(200.0)
        time.sleep(0.5)
        idle_ok = abs(axis.telemetry.actual_deg) < 5
        line(idle_ok, "без разрешения ось стоит",
             f"ушла на {axis.telemetry.actual_deg:+.1f}°")

        # 2. После разрешения — доезжает.
        loop.set_armed(True)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and abs(axis.telemetry.error_deg) > 3:
            time.sleep(0.05)
        reach_ok = abs(axis.telemetry.error_deg) <= 3
        line(reach_ok, "доезжает до цели",
             f"остаток {axis.telemetry.error_deg:+.1f}°")

        # 3. Защита молчит на исправной оси.
        quiet_ok = loop.error_text is None
        line(quiet_ok, "защита не срабатывает впустую", loop.error_text or "")
    finally:
        loop.stop()

    return idle_ok and reach_ok and quiet_ok


def check_portable() -> bool:
    """Ничего, что работает только на одной системе, вне platform_bits."""
    print("\nпереносимость:")
    done = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "check_portable.py")],
        capture_output=True, text=True, timeout=60)
    if done.returncode != 0:
        строки = done.stdout.strip().splitlines()
        return line(False, "различия систем собраны в одном месте",
                    строки[0] if строки else "")
    return line(True, "различия систем собраны в одном месте")


def check_gui() -> bool:
    """Окно управления должно собираться целиком.

    Проверки `--help` тут мало: разбор аргументов проходит, а окно потом
    падает на не определённом имени или на переменной, использованной до
    присваивания. Так подряд ушли в работу два падения — на сохранении
    середины и на кнопке исходного положения. Здесь окно действительно
    строится, с заглушкой вместо камеры, и само закрывается.
    """
    print("\nокно управления:")
    done = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "arm_control.py"), "--smoke", "2"],
        capture_output=True, timeout=180)
    if done.returncode != 0:
        tail = done.stderr.decode(errors="replace").strip().splitlines()
        return line(False, "окно собирается",
                    tail[-1] if tail else "ненулевой код возврата")
    return line(True, "окно собирается и закрывается")


def main() -> int:
    print(f"проверка проекта в {ROOT}\n")
    results = [check_doctests(), check_entry_points(), check_control_loop(),
               check_portable(), check_gui()]
    print()
    if all(results):
        print(f"{GREEN}всё в порядке{OFF}")
        return 0
    print(f"{RED}есть сбои — смотрите выше{OFF}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
