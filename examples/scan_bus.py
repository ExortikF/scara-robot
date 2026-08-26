"""Поиск привода на шине: перебор скорости, чётности и адреса ведомого.

    python examples/scan_bus.py /dev/cu.usbserial-A1

Читает P-003 (версия ПО, только чтение — безопасно) у каждого кандидата.
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
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING)
    first, _, last = args.ids.partition("-")
    ids = range(int(first), int(last or first) + 1)

    found = []
    for baud in BAUDRATES:
        for parity in PARITIES:
            for device_id in ids:
                serial = SerialConfig(
                    port=args.port, baudrate=baud, parity=parity,
                    timeout=0.15, retries=1, device_id=device_id,
                )
                try:
                    with ServoDrive(DriveConfig(serial=serial)) as drive:
                        version = drive.read_param(reg.P_SOFTWARE_VERSION)
                except (DriveError, OSError):
                    continue
                print(f"НАЙДЕН: {baud} {parity} id={device_id}  P-003={version}")
                found.append((baud, parity, device_id))

    if not found:
        print("ничего не найдено — проверьте A/B, терминатор 120 Ом и общий GND")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
