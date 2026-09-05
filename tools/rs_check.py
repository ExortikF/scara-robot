"""Проверка RealSense отдельно от всего остального.

    sudo .venv-rs/bin/python tools/rs_check.py

Открывает камеру тем же кодом, что и основная программа, и печатает
каждый шаг. Нужна, чтобы отделить «камера не открывается» от «не работает
всё остальное»: в основной программе камера — только один из десятка
узлов, и её отказ теряется среди прочего.
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")


def main() -> int:
    import argparse
    import os

    # Разбор аргументов нужен не ради самих аргументов, а ради --help:
    # по нему проверяется, что скрипт цел, и без него самопроверка
    # запускала бы настоящий опрос камеры и падала без прав root.
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--keep-context", action="store_true",
                        help="не отпускать контекст — воспроизводит ошибку "
                             "«No device connected» для сравнения")
    parser.parse_args()

    print("=== проверка RealSense ===\n")

    print(f"права root: {'да' if os.geteuid() == 0 else 'НЕТ — нужен sudo'}")
    if os.geteuid() != 0:
        print("\nЗапускайте так:")
        print("    sudo .venv-rs/bin/python tools/rs_check.py")
        return 1

    try:
        import pyrealsense2 as rs
    except ImportError:
        print("pyrealsense2 не установлен в этом окружении")
        print("Запускайте питоном из .venv-rs")
        return 1
    print("pyrealsense2: есть")

    import subprocess
    subprocess.run(["killall", "VDCAssistant"], capture_output=True, check=False)
    time.sleep(0.6)
    print("VDCAssistant: согнан")

    ctx = rs.context()
    devices = list(ctx.query_devices())
    print(f"устройств найдено: {len(devices)}")
    if not devices:
        print("\nКамеры нет на шине. Проверьте кабель — нужен USB 3.")
        return 1

    device = devices[0]
    keep_context = "--keep-context" in sys.argv
    print(f"  {device.get_info(rs.camera_info.name)}")
    try:
        print(f"  серийный номер: {device.get_info(rs.camera_info.serial_number)}")
        print(f"  прошивка: {device.get_info(rs.camera_info.firmware_version)}")
    except Exception:
        pass

    print("\nцветные режимы, которые заявляет камера:")
    modes = set()
    for sensor in device.sensors:
        try:
            profiles = list(sensor.profiles)
        except Exception as exc:
            print(f"  сенсор не опрашивается: {exc}")
            continue
        for profile in profiles:
            if "color" not in str(profile.stream_type()):
                continue
            try:
                video = profile.as_video_stream_profile()
            except Exception:
                continue
            modes.add((video.width(), video.height(), profile.fps()))
    if modes:
        for mode in sorted(modes)[:8]:
            print(f"  {mode[0]}x{mode[1]}@{mode[2]}")
        if len(modes) > 8:
            print(f"  ... всего {len(modes)}")
    else:
        print("  ни одного — сенсор занят или камера отдаёт только глубину")

    # Контекст обязан умереть до создания конвейера: пока он жив,
    # устройство числится за ним, а rs.pipeline() заводит свой контекст и
    # камеру не находит — отсюда «No device connected» сразу после
    # успешного перечисления. Ровно на этом спотыкалась основная
    # программа, и здесь та же ловушка воспроизводится один в один.
    if not keep_context:
        import gc
        devices.clear()
        del devices, device, ctx
        gc.collect()
        print("\nконтекст отпущен (камера освобождена)")
    else:
        print("\nконтекст НАМЕРЕННО удержан — ожидается «No device connected»")

    print("\nпробую открыть цветной поток:")
    for w, h, f in [(640, 480, 30), (640, 480, 15), (848, 480, 30),
                    (1280, 720, 30)]:
        print(f"  {w}x{h}@{f} ... ", end="", flush=True)
        try:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
            pipeline.start(cfg)
        except Exception as exc:
            print(f"не стартовал: {exc}")
            continue
        try:
            frames = pipeline.wait_for_frames(2000)
            colour = frames.get_color_frame()
            ok = bool(colour)
        except Exception as exc:
            print(f"кадров нет: {exc}")
            try:
                pipeline.stop()
            except Exception:
                pass
            continue
        pipeline.stop()
        if ok:
            print("КАДР ЕСТЬ")
            print(f"\nКамера работает. В основной программе используйте "
                  f"этот режим.")
            return 0
        print("поток открылся, но кадр пустой")

    print("\nНи один режим не отдал кадр.")
    print("Что проверить:")
    print("  * не запущено ли ещё что-то:  pgrep -fl arm_control")
    print("  * не держит ли камеру приложение (Photo Booth, FaceTime, Zoom)")
    print("  * вынуть и вставить кабель, обязательно USB 3")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
