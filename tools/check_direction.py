"""Проверка согласованности знаков: команда, энкодер, сообщаемая скорость.

Контур позиционирования держится на трёх допущениях:

1. команда «вперёд» увеличивает показание энкодера;
2. привод сообщает скорость с тем же знаком, что и направление счёта;
3. галка «инверсия энкодера» согласована с первыми двумя.

Если хоть одно нарушено, обратная связь становится положительной, и ось
вместо подхода к цели уходит от неё, разворачивается на противоположной
стороне окружности и уходит обратно. Со стороны это выглядит как «просто
крутится влево-вправо».

Скрипт проверяет все три допущения напрямую: коротко двигает вал в каждую
сторону на минимальной скорости и смотрит, что происходит с регистрами.

    python tools/check_direction.py --port /dev/cu.usbserial-XXXX --confirm

ВАЖНО: скрипт ДВИГАЕТ ВАЛ. Ось должна быть разгружена, муфта снята,
аварийный останов под рукой.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import DriveConfig, DriveError, SerialConfig, ServoDrive, TcpConfig
from scara_drive import registers as reg
from scara_drive.discovery import guess_port


def to_signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def probe(drive: ServoDrive, *, reverse: bool, speed_rpm: int,
          seconds: float) -> tuple[int, list[int]]:
    """Двинуть вал и вернуть (прирост счётчика, замеры сообщаемой скорости)."""
    before = drive.read_position()
    speeds: list[int] = []

    drive.run(speed_rpm, reverse=reverse)
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            speeds.append(to_signed16(drive._read(reg.REG_ACTUAL_SPEED, 1)[0]))
            time.sleep(0.05)
    finally:
        drive._write(drive.config.cmd_register, reg.CMD_HOLD)

    time.sleep(0.3)                     # дать валу остановиться
    after = drive.read_position()
    return after - before, speeds


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--parity", default="E", choices=["N", "E", "O"])
    p.add_argument("--id", type=int, default=1)
    p.add_argument("--host", help="Modbus TCP вместо RS-485")
    p.add_argument("--speed", type=int, default=1, help="об/мин на пробу")
    p.add_argument("--seconds", type=float, default=2.0)
    p.add_argument("--cpr", type=int, default=131072, help="импульсов на оборот")
    p.add_argument("--confirm", action="store_true",
                   help="подтвердить, что ось разгружена и вал можно крутить")
    args = p.parse_args()

    if not args.confirm:
        print(__doc__)
        print("Запустите с --confirm, когда ось будет разгружена.")
        return 1

    if args.host:
        config = DriveConfig(tcp=TcpConfig(host=args.host, device_id=args.id))
    else:
        port = args.port or guess_port()
        if port is None:
            print("не нашёл переходник — укажите --port", file=sys.stderr)
            return 1
        config = DriveConfig(serial=SerialConfig(
            port=port, baudrate=args.baud, parity=args.parity, device_id=args.id))

    drive = ServoDrive(config)
    drive.connect()
    try:
        drive.write_param(reg.P_SERVO_ENABLE, 1)
        drive._write(drive.config.cmd_register, reg.CMD_HOLD)
        time.sleep(0.2)

        print(f"проба: {args.speed} об/мин, по {args.seconds:g} с в каждую сторону\n")
        fwd_counts, fwd_speeds = probe(
            drive, reverse=False, speed_rpm=args.speed, seconds=args.seconds)
        rev_counts, rev_speeds = probe(
            drive, reverse=True, speed_rpm=args.speed, seconds=args.seconds)
    except DriveError as exc:
        print(f"ошибка обмена: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            drive._write(drive.config.cmd_register, reg.CMD_STOP)
        finally:
            drive.close()

    def summarize(name: str, counts: int, speeds: list[int]) -> float:
        degrees = counts * 360.0 / args.cpr
        moving = [s for s in speeds if s != 0]
        median = sorted(moving)[len(moving) // 2] if moving else 0
        print(f"{name}:")
        print(f"  счётчик энкодера  {counts:+d} имп. = {degrees:+.2f}°")
        print(f"  сообщаемая скорость  медиана {median:+d} об/мин "
              f"(замеров {len(speeds)}, ненулевых {len(moving)})")
        return degrees

    fwd_deg = summarize("ВПЕРЁД (0x2222)", fwd_counts, fwd_speeds)
    print()
    rev_deg = summarize("НАЗАД  (0x1111)", rev_counts, rev_speeds)

    print("\n" + "=" * 60)
    problems = []

    if abs(fwd_deg) < 0.05 and abs(rev_deg) < 0.05:
        print("вал не двигался. Привод не разрешён, авария или не тот режим —")
        print(f"проверьте код аварии: examples/mb.py read 0x{reg.REG_ALARM:04X}")
        return 1

    if fwd_deg < 0:
        problems.append(
            "команда ВПЕРЁД уменьшает показание энкодера.\n"
            "    Это и есть положительная обратная связь: контур гонит ось\n"
            "    прочь от цели, на другой стороне окружности ошибка меняет\n"
            "    знак, ось разворачивается — и так по кругу.\n"
            "    Лечится галкой «инверсия энкодера» в GUI."
        )

    if fwd_deg > 0 and rev_deg > 0 or fwd_deg < 0 and rev_deg < 0:
        problems.append(
            "обе команды двигают вал в одну сторону — направление не\n"
            "    переключается. Проверьте значения команд в регистре 0x1010."
        )

    fwd_moving = [s for s in fwd_speeds if s != 0]
    if fwd_moving:
        speed_sign = 1 if sorted(fwd_moving)[len(fwd_moving) // 2] > 0 else -1
        count_sign = 1 if fwd_deg > 0 else -1
        if speed_sign != count_sign:
            problems.append(
                "знак сообщаемой скорости противоположен направлению счёта.\n"
                "    Составляющая Kd тогда не гасит движение, а подкачивает —\n"
                "    чем больше Kd, тем сильнее раскачка. До выяснения\n"
                "    поставьте Kd = 0."
            )

    if problems:
        print("НАЙДЕНО:")
        for i, item in enumerate(problems, 1):
            print(f"  {i}. {item}")
    else:
        print("знаки согласованы: вперёд увеличивает счёт, назад уменьшает,")
        print("сообщаемая скорость совпадает по знаку с направлением счёта.")
        print("\nЗначит раскачка не от знаков, а от бюджета времени контура:")
        print("  python tools/pid_budget.py --gap 0.005 --sleep 0.020")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
