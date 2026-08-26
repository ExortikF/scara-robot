"""Чтение и запись регистров привода по Modbus RTU. Рабочий инструмент наладки.

    python examples/mb.py read  P-004
    python examples/mb.py read  40077
    python examples/mb.py read  0x1011
    python examples/mb.py read  P-028 --count 3
    python examples/mb.py write 0x1011 8738
    python examples/mb.py write P-076 100 --verify
    python examples/mb.py watch 0x1011 --count 2

Порт определяется сам, если переходник в ноутбуке один; иначе --port.

Адрес понимается в трёх видах:

    P-004    параметр из руководства
    40077    нотация 4xxxx (= 40001 + номер параметра)
    0x1011   сырой адрес регистра; 4113 в десятичном виде тоже работает
"""

import argparse
import logging
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import DriveConfig, DriveError, SerialConfig, ServoDrive, TcpConfig
from scara_drive import registers as reg
from scara_drive.discovery import guess_port, list_ports

PARAM_RE = re.compile(r"^[Pp]-?(\d{1,3})$")


def parse_address(text: str) -> tuple[int, str]:
    """Строку адреса -> (адрес для PDU, как его показывать)."""
    match = PARAM_RE.match(text)
    if match:
        number = int(match.group(1))
        return reg.param_addr(number), f"P-{number:03d}"

    value = int(text, 0)            # 0 -> распознаёт 0x, 0b, 0o
    if value >= reg.PARAM_BASE:     # 40001 и выше — нотация 4xxxx
        number = value - reg.PARAM_BASE
        return number, f"{value} (P-{number:03d})"
    return value, f"{value} (0x{value:04X})"


def build_config(args) -> DriveConfig:
    if args.host:
        return DriveConfig(tcp=TcpConfig(host=args.host, port=args.tcp_port,
                                         device_id=args.id))
    port = args.port or guess_port()
    if port is None:
        found = list_ports()
        raise SystemExit(
            "не понял, какой порт брать — укажите --port.\n"
            + ("кандидаты:\n  " + "\n  ".join(str(p) for p in found)
               if found else "переходник не найден, запустите examples/ports.py")
        )
    return DriveConfig(serial=SerialConfig(
        port=port, baudrate=args.baud, parity=args.parity, device_id=args.id,
    ))


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", choices=["read", "write", "watch"])
    p.add_argument("address")
    p.add_argument("value", nargs="?", type=lambda s: int(s, 0))
    p.add_argument("--count", type=int, default=1, help="сколько регистров подряд")
    p.add_argument("--verify", action="store_true",
                   help="после записи прочитать обратно и сверить")
    p.add_argument("--interval", type=float, default=0.2, help="период watch, с")

    p.add_argument("--port", help="/dev/cu.usbserial-XXXX")
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--parity", default="N", choices=["N", "E", "O"])
    p.add_argument("--id", type=int, default=1, help="адрес ведомого")
    p.add_argument("--host", help="Modbus TCP через шлюз вместо RS-485")
    p.add_argument("--tcp-port", type=int, default=502)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING,
                        format="%(levelname)s %(message)s")

    address, label = parse_address(args.address)
    config = build_config(args)

    # Тут ServoDrive как контекстный менеджер не годится: на выходе он шлёт
    # стоп в регистр команд, а инструменту наладки самовольничать нельзя.
    drive = ServoDrive(config)
    drive.connect()
    try:
        if args.action == "read":
            values = drive._read(address, args.count)
            for i, value in enumerate(values):
                where = label if i == 0 else f"+{i}"
                print(f"{where} = {value}  (0x{value:04X})")

        elif args.action == "write":
            if args.value is None:
                p.error("для write нужно значение")
            drive._write(address, args.value)
            print(f"{label} <- {args.value}  (0x{args.value:04X})")
            if args.verify:
                back = drive._read(address, 1)[0]
                if back == args.value:
                    print("сверка: ок")
                else:
                    print(f"СВЕРКА НЕ ПРОШЛА: прочитано {back}, ожидалось {args.value}")
                    print("частая причина — пароль: P-000 (по умолчанию 315), "
                          "а P-001/P-002/P-099 требуют 385 или 316")
                    return 1

        else:  # watch
            print(f"{label}, каждые {args.interval:g} с, Ctrl-C для выхода")
            try:
                while True:
                    values = drive._read(address, args.count)
                    print("  ".join(f"{v:6d} 0x{v:04X}" for v in values), end="\r")
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                print()
    except DriveError as exc:
        print(f"ошибка: {exc}", file=sys.stderr)
        return 1
    finally:
        drive.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
