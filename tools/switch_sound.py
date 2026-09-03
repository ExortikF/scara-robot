"""Звук на компьютере при срабатывании концевика.

Слушает последовательный порт ESP32 и на каждое замыкание проигрывает
звук. Работает вместе со скетчем ``arduino/limit_switch_test``.

    python tools/switch_sound.py                 звук по умолчанию
    python tools/switch_sound.py --list          что вообще доступно
    python tools/switch_sound.py --sound Frog
    python tools/switch_sound.py --say "ой" --voice Bubbles
    python tools/switch_sound.py --random        каждый раз новый

ВАЖНО: порт может держать только одна программа. Закройте монитор порта
в Arduino IDE, иначе скрипт его не откроет.
"""

from __future__ import annotations

import argparse
import random
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive.discovery import (PortBusyError, PortGoneError,
                                   list_ports, open_serial)

SOUND_DIR = Path("/System/Library/Sounds")

# Отобраны по звучанию: короткие и с характером. Длинные и унылые
# (Basso, Blow, Morse) в список не берём — на каждом нажатии надоедают.
FUNNY_SOUNDS = ["Frog", "Sosumi", "Submarine", "Purr", "Pop", "Bottle", "Funk"]

FUNNY_PHRASES = [
    "оба-на", "приехали", "опа", "туда нельзя", "ай", "стоп машина",
    "куда поехал", "приплыли",
]


def available_sounds() -> list[str]:
    if not SOUND_DIR.is_dir():
        return []
    return sorted(p.stem for p in SOUND_DIR.glob("*.aiff"))


def play_sound(name: str) -> None:
    path = SOUND_DIR / f"{name}.aiff"
    # Не ждём окончания: концевик могут щёлкать быстрее, чем играет звук,
    # и блокировка здесь превратилась бы в отставание от железа.
    subprocess.Popen(["afplay", str(path)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def speak(text: str, voice: str | None) -> None:
    command = ["say"]
    if voice:
        command += ["-v", voice]
    command.append(text)
    subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="порт платы, по умолчанию определяется сам")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--sound", help="имя системного звука")
    p.add_argument("--say", help="произнести фразу вместо звука")
    p.add_argument("--voice", help="голос для --say, например Bubbles")
    p.add_argument("--random", action="store_true",
                   help="каждый раз новый звук или фраза")
    p.add_argument("--release", action="store_true",
                   help="звучать и на размыкание тоже")
    p.add_argument("--list", action="store_true", help="показать варианты и выйти")
    args = p.parse_args()

    if args.list:
        print("системные звуки (--sound):")
        for name in available_sounds():
            mark = "  ← смешной" if name in FUNNY_SOUNDS else ""
            print(f"  {name}{mark}")
        print("\nголоса (--voice): Bubbles, Boing, Jester, Trinoids, Zarvox,")
        print("                  Cellos, Wobble, Superstar, Bells, Milena")
        print("\nпослушать:  afplay /System/Library/Sounds/Frog.aiff")
        print("            say -v Bubbles 'оба-на'")
        return 0

    port = args.port
    if port is None:
        # Плата ESP32 — это USB-переходник, тот же поиск, что и для RS-485.
        found = list_ports()
        if not found:
            # Ноль кандидатов и несколько кандидатов — разные беды, и
            # советы у них разные. Раньше оба случая выдавали одно и то же
            # «не понял, какой порт брать», что для пустого списка неверно.
            print("плата не подключена: в системе нет ни одного "
                  "USB-порта.\n", file=sys.stderr)
            print("проверьте по порядку:", file=sys.stderr)
            print("  1. воткнута ли плата;", file=sys.stderr)
            print("  2. КАБЕЛЬ — многие идут только для зарядки, без "
                  "сигнальных жил.", file=sys.stderr)
            print("     Плата с таким кабелем светится, но компьютер её "
                  "не видит;", file=sys.stderr)
            print("  3. воткните напрямую, без хаба;", file=sys.stderr)
            print("  4. видит ли её система на уровне USB:", file=sys.stderr)
            print("     ioreg -p IOUSB -l -w 0 | grep -i 'USB Product'",
                  file=sys.stderr)
            return 1
        if len(found) > 1:
            print("несколько подходящих портов — укажите нужный через "
                  "--port:", file=sys.stderr)
            for item in found:
                print(f"  {item}", file=sys.stderr)
            return 1
        port = found[0].device

    def fire(closed: bool) -> None:
        if args.say is not None:
            speak(random.choice(FUNNY_PHRASES) if args.random else args.say,
                  args.voice)
        elif args.random:
            play_sound(random.choice(FUNNY_SOUNDS))
        else:
            play_sound(args.sound or "Frog")

    # Порт открываем до баннера: иначе сообщение об ошибке печатается
    # после жизнерадостного «слушаю...», и читать это неудобно.
    try:
        ser = open_serial(port, baudrate=args.baud, timeout=0.2)
    except (PortGoneError, PortBusyError) as exc:
        print(exc, file=sys.stderr)
        return 1

    print(f"слушаю {port} на {args.baud}")
    if args.say is not None:
        print(f"говорю: {'случайную фразу' if args.random else args.say}")
    else:
        print(f"звук: {'случайный' if args.random else (args.sound or 'Frog')}")
    print("щёлкайте концевиком. Ctrl-C для выхода.\n")

    hits = 0
    try:
        with ser:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip()
                if not line:
                    continue
                print(line)

                # Скетч печатает состояние словами — на них и опираемся.
                if "ЗАМКНУТ" in line:
                    hits += 1
                    fire(True)
                elif args.release and "разомкнут" in line:
                    fire(False)
    except KeyboardInterrupt:
        print(f"\nвсего срабатываний: {hits}")
    except (PortGoneError, PortBusyError) as exc:
        print(f"\n{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
