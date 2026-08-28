"""Проверка электрического состояния шины RS-485.

Отвечает на вопрос «линия чистая или шумит», который по логам Modbus видно
плохо: там всё выглядит как «привод не ответил».

Тест простой: открыть порт и **ничего не передавая** послушать линию. На
исправной шине в покое не должно прийти ни одного байта. Если байты идут —
приёмник видит шум и раскладывает его на кадры. Обычно это `0xff` или `0x00`
подряд.

Почему это ломает обмен: пока байты продолжают приходить, чтение ответа не
может завершиться по таймауту — оно бесконечно читает мусор. Программа
выглядит зависшей, хотя привод при этом отвечает корректно.

    python tools/bus_health.py --port /dev/cu.usbserial-XXXX
"""

from __future__ import annotations

import argparse
import collections
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive.discovery import guess_port


def listen(port: str, baud: int, parity: str, seconds: float) -> bytes:
    from scara_drive.discovery import open_serial

    with open_serial(port, baudrate=baud, parity=parity, bytesize=8,
                     stopbits=1, timeout=0.1) as ser:
        ser.reset_input_buffer()
        collected = bytearray()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            chunk = ser.read(4096)
            if chunk:
                collected.extend(chunk)
        return bytes(collected)


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--parity", default="E", choices=["N", "E", "O"])
    p.add_argument("--seconds", type=float, default=3.0)
    args = p.parse_args()

    port = args.port or guess_port()
    if port is None:
        print("не нашёл переходник — укажите --port", file=sys.stderr)
        return 1

    print(f"слушаю {port} ({args.baud} 8{args.parity}1) {args.seconds:g} с, "
          f"ничего не передавая...\n")
    data = listen(port, args.baud, args.parity, args.seconds)

    rate = len(data) / args.seconds
    print(f"принято байт: {len(data)}  ({rate:.0f} байт/с)")

    if not data:
        print("\nЛИНИЯ ЧИСТАЯ. В покое шума нет — электрика в порядке.")
        print("Если обмен всё равно срывается, причина не в линии:")
        print("  проверьте адреса, скорость, чётность и питание приводов.")
        return 0

    counts = collections.Counter(data)
    print("\nчаще всего встречаются:")
    for value, count in counts.most_common(4):
        print(f"  0x{value:02X} — {count} раз ({count / len(data) * 100:.0f}%)")

    dominant, dominant_count = counts.most_common(1)[0]
    share = dominant_count / len(data)

    print("\nЛИНИЯ ШУМИТ — это и есть причина зависаний.")
    print("Пока байты идут непрерывно, чтение ответа не завершается по")
    print("таймауту: оно читает мусор, и программа выглядит повисшей.\n")

    if dominant in (0x00, 0xFF) and share > 0.8:
        print(f"Поток почти целиком из 0x{dominant:02X} — характерный признак")
        print("линии без смещения (bias). В покое, когда никто не передаёт,")
        print("дифференциальная пара болтается около нуля, приёмник трактует")
        print("это как непрерывный поток бит.\n")
        print("Что делать по железу, по порядку:")
        print("  1. терминаторы 120 Ом на ДВУХ концах сегмента, не в середине;")
        print("  2. резисторы смещения: A через 560 Ом -> +5 В, B через 560 Ом")
        print("     -> GND. На многих переходниках они есть, но не распаяны —")
        print("     ищите перемычки с подписью bias / pull-up / fail-safe;")
        print("  3. общий GND между переходником и всеми приводами;")
        print("  4. витая пара на A/B, отводы к приводам короткие.")
        print("\nЧасть переходников FTDI имеет встроенный fail-safe bias —")
        print("тогда достаточно терминаторов и земли.")
    else:
        print("Состав шума неоднородный — похоже не на плавающую линию,")
        print("а на наводку или на второе устройство, которое передаёт в шину.")
        print("Проверьте, не запущена ли параллельно другая программа с этим")
        print("портом, и разнесите сигнальный кабель с силовым.")

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
