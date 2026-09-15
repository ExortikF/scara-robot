"""Звук на компьютере при срабатывании концевика.

Слушает последовательный порт ESP32 и на каждое замыкание проигрывает
звук. Работает вместе со скетчем ``arduino/limit_switch_test``.

    python tools/switch_sound.py                 звук по умолчанию
    python tools/switch_sound.py --list          что вообще доступно
    python tools/switch_sound.py --sound squeak
    python tools/switch_sound.py --random        каждый раз новый
    python tools/switch_sound.py --release       и на размыкание тоже
    python tools/switch_sound.py --say "ой" --voice Bubbles

Звуки лежат в папке sounds/ рядом с проектом. Чтобы добавить свой,
просто положите туда файл — wav, aiff, mp3 или m4a.

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

SOUND_DIR = Path(__file__).resolve().parents[1] / "sounds"

# Порядок задаёт и порядок в --list, и вероятность при --random.
# Тромбон длиннее прочих в пять раз, поэтому в случайную выдачу не идёт:
# при частых щелчках он накладывался бы сам на себя. Вызывается только
# по имени — как звук для размыкания он как раз хорош.
LONG_SOUNDS = {"trombone"}

FUNNY_PHRASES = [
    "оба-на", "приехали", "опа", "туда нельзя", "ай", "стоп машина",
    "куда поехал", "приплыли",
]


def available_sounds() -> list[str]:
    if not SOUND_DIR.is_dir():
        return []
    return sorted(p.stem for p in SOUND_DIR.iterdir()
                  if p.suffix.lower() in (".wav", ".aiff", ".mp3", ".m4a"))


def sound_path(name: str) -> Path | None:
    for candidate in SOUND_DIR.glob(f"{name}.*"):
        if candidate.suffix.lower() in (".wav", ".aiff", ".mp3", ".m4a"):
            return candidate
    return None


def play_sound(name: str) -> None:
    path = sound_path(name)
    if path is None:
        print(f"нет такого звука: {name}", file=sys.stderr)
        return
    # Не ждём окончания: концевик могут щёлкать быстрее, чем играет звук,
    # и блокировка здесь превратилась бы в отставание от железа.
    from scara_drive.platform_bits import проиграть_звук

    проиграть_звук(path)


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
    p.add_argument("--release-sound", default="trombone",
                   help="что играть на размыкание при --release")
    p.add_argument("--list", action="store_true", help="показать варианты и выйти")
    args = p.parse_args()

    if args.list:
        names = available_sounds()
        if not names:
            print(f"в папке {SOUND_DIR} нет звуковых файлов")
            return 1
        print(f"звуки из {SOUND_DIR.name}/ (--sound):")
        for name in names:
            mark = "  — длинный, в --random не участвует" if name in LONG_SOUNDS else ""
            print(f"  {name}{mark}")
        print("\nголоса (--voice): Bubbles, Boing, Jester, Trinoids, Zarvox,")
        print("                  Cellos, Wobble, Superstar, Bells, Milena")
        print(f"\nпослушать:  afplay {SOUND_DIR}/splat.wav")
        print("            say -v Bubbles 'оба-на'")
        print(f"\nсвои звуки просто кладите в {SOUND_DIR.name}/ — "
              "wav, aiff, mp3 или m4a")
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

    names = available_sounds()
    if not names and args.say is None:
        print(f"в папке {SOUND_DIR} нет звуковых файлов", file=sys.stderr)
        return 1

    if args.sound and args.sound not in names:
        print(f"нет звука «{args.sound}». Доступны: {', '.join(names)}",
              file=sys.stderr)
        return 1

    shuffle_pool = [n for n in names if n not in LONG_SOUNDS] or names
    default_sound = args.sound or ("splat" if "splat" in names else names[0])

    def fire(closed: bool) -> None:
        if args.say is not None:
            speak(random.choice(FUNNY_PHRASES) if args.random else args.say,
                  args.voice)
        elif args.random:
            play_sound(random.choice(shuffle_pool))
        else:
            play_sound(default_sound if closed else args.release_sound)

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
        print(f"звук: {'случайный' if args.random else default_sound}")
        if args.release:
            print(f"на размыкание: {args.release_sound}")
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
