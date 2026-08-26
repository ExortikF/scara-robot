"""Толчковый прогон в скоростном режиме на разгруженной оси.

    python examples/jog.py /dev/tty.usbserial-A1 --speed 100 --seconds 2

Перед первым запуском снимите нагрузку с оси: значение команды «назад»
взято из захвата обмена и не подтверждено руководством.
"""

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import DriveConfig, SerialConfig, ServoDrive
from scara_drive import registers as reg


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("port")
    p.add_argument("--id", type=int, default=1)
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--parity", default="N", choices=["N", "E", "O"])
    p.add_argument("--speed", type=int, default=100)
    p.add_argument("--seconds", type=float, default=2.0)
    p.add_argument("--reverse", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    serial = SerialConfig(
        port=args.port, baudrate=args.baud, parity=args.parity, device_id=args.id
    )
    with ServoDrive(DriveConfig(serial)) as drive:
        mode = drive.read_param(reg.P_CONTROL_MODE)
        print(f"P-004 (режим управления) = {mode}")
        if mode != reg.ControlMode.SPEED:
            print("привод не в скоростном режиме — через Modbus он не поедет")
            print(f"    задайте P-004 = {reg.ControlMode.SPEED} с панели привода")
            return 1

        drive.run(args.speed, reverse=args.reverse)
        time.sleep(args.seconds)
        # stop() вызывается в __exit__ в любом случае, в т.ч. по Ctrl-C
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
