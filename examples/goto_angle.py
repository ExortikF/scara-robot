"""Приехать в заданный угол. Задал — доехал.

    python examples/goto_angle.py 90
    python examples/goto_angle.py 90 180 270 0        # по очереди
    python examples/goto_angle.py 45 --tolerance 0.5 --max-rpm 30
    python examples/goto_angle.py 90 --set-zero       # сначала принять текущее за 0
    python examples/goto_angle.py 90 --simulate       # без железа, проверить логику

Настройки связи и энкодера берутся из gui/c30_servo_settings.json — того же
файла, что использует GUI. Любую можно перекрыть ключом командной строки.
"""

import argparse
import json
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from scara_drive import (
    C30AngleIO, DriveConfig, DriveError, GotoController, GotoError, GotoProfile,
    SerialConfig, ServoDrive, TcpConfig,
)
from scara_drive.discovery import guess_port

SETTINGS = ROOT / "gui" / "c30_servo_settings.json"


def load_settings() -> dict:
    if not SETTINGS.is_file():
        return {}
    try:
        return json.loads(SETTINGS.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"предупреждение: не читается {SETTINGS.name}: {exc}", file=sys.stderr)
        return {}


def build_parser(saved: dict) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("angles", nargs="+", type=float, help="целевые углы, 0..360")

    p.add_argument("--port", default=saved.get("port"))
    p.add_argument("--baud", type=int, default=int(saved.get("baudrate", 19200)))
    p.add_argument("--parity", default=saved.get("parity", "E"), choices=["N", "E", "O"])
    p.add_argument("--id", type=int, default=int(saved.get("slave_id", 1)))
    p.add_argument("--host", help="Modbus TCP через шлюз вместо RS-485")

    p.add_argument("--cpr", type=int, default=int(saved.get("counts_per_rev", 131072)),
                   help="отсчётов энкодера на оборот")
    p.add_argument("--zero", type=int, default=int(saved.get("zero_offset_counts", 0)),
                   help="программный ноль в отсчётах")
    p.add_argument("--invert", action="store_true",
                   default=bool(saved.get("invert_encoder", False)))
    p.add_argument("--set-zero", action="store_true",
                   help="принять текущее положение за 0° перед движением")

    p.add_argument("--tolerance", type=float, default=1.0, help="допуск, градусы")
    p.add_argument("--max-rpm", type=int, default=20)
    p.add_argument("--min-rpm", type=int, default=1)
    p.add_argument("--timeout", type=float, default=60.0)

    p.add_argument("--simulate", action="store_true", help="без железа")
    p.add_argument("-q", "--quiet", action="store_true")
    return p


def make_simulated(args):
    sys.path.insert(0, str(ROOT / "tests"))
    from test_goto import SimulatedDrive

    drive = SimulatedDrive(angle=0.0)
    return drive, drive.clock, drive.sleep


def main() -> int:
    saved = load_settings()
    args = build_parser(saved).parse_args()

    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")

    profile = GotoProfile(
        tolerance_deg=args.tolerance,
        max_rpm=args.max_rpm,
        min_rpm=args.min_rpm,
        timeout_s=args.timeout,
    )

    if args.simulate:
        io, clock, sleep = make_simulated(args)
        print("режим симуляции: железо не используется\n")
        controller = GotoController(io, profile, clock=clock, sleep=sleep)
        return drive_to(controller, io, args)

    if args.host:
        config = DriveConfig(tcp=TcpConfig(host=args.host, device_id=args.id))
    else:
        port = args.port or guess_port()
        if not port:
            print("не нашёл порт — укажите --port "
                  "(список: python examples/ports.py)", file=sys.stderr)
            return 1
        config = DriveConfig(serial=SerialConfig(
            port=port, baudrate=args.baud, parity=args.parity, device_id=args.id,
        ))

    drive = ServoDrive(config)
    drive.connect()
    try:
        io = C30AngleIO(drive, counts_per_rev=args.cpr,
                        zero_offset_counts=args.zero, invert=args.invert)

        alarm = io.read_alarm()
        if alarm:
            print(f"привод в аварии, код 0x{alarm:04X} — сбросьте до движения",
                  file=sys.stderr)
            return 1

        if args.set_zero:
            offset = io.set_zero_here()
            print(f"текущее положение принято за 0° (offset RAW={offset})")

        io.enable()
        controller = GotoController(io, profile)
        return drive_to(controller, io, args)
    except DriveError as exc:
        print(f"ошибка связи: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            io.hold()
        except (DriveError, NameError):
            pass
        drive.close()


def drive_to(controller, io, args) -> int:
    print(f"старт: {io.read_angle():.2f}°, допуск {args.tolerance}°\n")
    failures = 0

    for target in args.angles:
        # Перерисовку строки на месте включаем только в живом терминале:
        # в файл или в конвейер это свалилось бы в одну кашу.
        live = sys.stdout.isatty() and not args.quiet

        def progress(angle, error, _t=target):
            if live:
                print(f"  цель {_t:6.1f}° | сейчас {angle:7.2f}° | "
                      f"ошибка {error:+7.2f}°", end="\r", flush=True)

        try:
            result = controller.goto(target, on_progress=progress)
        except GotoError as exc:
            print(f"\nцель {target:.1f}°: {exc}", file=sys.stderr)
            failures += 1
            continue
        if live:
            print(" " * 60, end="\r")
        print(f"цель {target:6.1f}° -> {result}")

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
