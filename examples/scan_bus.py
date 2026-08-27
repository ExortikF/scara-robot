"""Поиск приводов на шине: перебор скорости, чётности и адреса ведомого.

Параметры связи неизвестны — перебрать всё:

    python examples/scan_bus.py /dev/cu.usbserial-A1

Параметры известны, нужно просто найти все приводы (быстро):

    python examples/scan_bus.py /dev/cu.usbserial-A1 --baud 19200 --parity E --ids 1-32

Читает P-003 (версия ПО, только чтение — безопасно) у каждого кандидата,
а у найденных заодно показывает P-181 (собственный адрес привода).
"""

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import DriveConfig, DriveError, SerialConfig, ServoDrive
from scara_drive import registers as reg

BAUDRATES = (9600, 19200, 38400, 57600, 115200)
PARITIES = ("N", "E", "O")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("port")
    p.add_argument("--ids", default="1-8", help="диапазон адресов, напр. 1-32")
    p.add_argument("--baud", type=int, help="фиксировать скорость (иначе перебор)")
    p.add_argument("--parity", choices=["N", "E", "O"],
                   help="фиксировать чётность (иначе перебор)")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING)
    first, _, last = args.ids.partition("-")
    ids = range(int(first), int(last or first) + 1)

    bauds = [args.baud] if args.baud else BAUDRATES
    parities = [args.parity] if args.parity else PARITIES
    print(f"опрашиваю адреса {ids.start}..{ids.stop - 1} "
          f"на {len(bauds)} скоростях x {len(parities)} чётностях\n")

    found = []
    for baud in bauds:
        for parity in parities:
            for device_id in ids:
                serial = SerialConfig(
                    port=args.port, baudrate=baud, parity=parity,
                    timeout=0.15, retries=1, device_id=device_id,
                )
                try:
                    # Контекстный менеджер тут не годится: на выходе он шлёт
                    # команду останова, а сканеру трогать чужие приводы нельзя.
                    drive = ServoDrive(DriveConfig(serial=serial))
                    drive.connect()
                    try:
                        version = drive.read_param(reg.P_SOFTWARE_VERSION)
                        own_id = drive.read_param(reg.P_COMM_ID)
                    finally:
                        drive.close()
                except (DriveError, OSError):
                    continue
                print(f"НАЙДЕН: id={device_id}  {baud} {parity}  "
                      f"P-003={version}  P-181={own_id}")
                found.append((baud, parity, device_id))

    if not found:
        print("ничего не найдено — проверьте A/B, терминатор 120 Ом и общий GND")
        return 1

    print(f"\nвсего приводов: {len(found)}")
    if len(found) > 1:
        print("адреса:", ", ".join(str(item[2]) for item in found))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
