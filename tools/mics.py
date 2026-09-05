"""Список микрофонов и проверка, что голос вообще слышен.

    python tools/mics.py            показать микрофоны
    python tools/mics.py --listen 5 послушать 5 секунд и показать уровень

Второе нужно, когда команды не срабатывают: сразу видно, приходит ли звук
и достаёт ли он до порога распознавания.
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--listen", type=float, metavar="СЕК",
                   help="послушать и показать уровень звука")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--level", type=int, default=300, help="порог тишины")
    args = p.parse_args()

    try:
        import sounddevice as sd
    except ImportError:
        print("нет sounddevice — запускайте питоном из .venv-rs")
        return 1

    if args.listen is None:
        print("микрофоны:")
        default = sd.default.device[0]
        for index, dev in enumerate(sd.query_devices()):
            if dev["max_input_channels"] <= 0:
                continue
            mark = "  <- системный по умолчанию" if index == default else ""
            print(f"  --audio-device {index}   {dev['name']}{mark}")
        print("\nпослушать выбранный:")
        print("  python tools/mics.py --listen 5 --device 0")
        return 0

    import array

    peak = 0.0
    loud = 0
    total = 0

    def feed(indata, frames, t, status):
        nonlocal peak, loud, total
        samples = array.array("h")
        samples.frombytes(bytes(indata))
        if not samples:
            return
        level = sum(abs(s) for s in samples) / len(samples)
        peak = max(peak, level)
        total += 1
        if level >= args.level:
            loud += 1

    print(f"слушаю микрофон {args.device} {args.listen:g} с — говорите "
          f"«стоп», «старт», «нули»...")
    with sd.RawInputStream(samplerate=16000, blocksize=4000,
                           device=args.device, dtype="int16", channels=1,
                           callback=feed):
        time.sleep(args.listen)

    print(f"\nкусков всего {total}, из них громче порога {loud}")
    print(f"самый громкий уровень: {peak:.0f}   (порог {args.level})")
    if peak < args.level:
        print("\nЗвук до порога не доходит. Варианты:")
        print("  * не тот микрофон — попробуйте другой --device;")
        print("  * не выдан доступ: Системные настройки -> Конфиденциальность")
        print("    -> Микрофон, потом ПЕРЕЗАПУСТИТЬ терминал;")
        print(f"  * говорите ближе или снизьте порог: --level {int(peak * 0.6)}")
        return 1
    if loud == 0:
        print("\nЗвук есть, но до порога не дотягивает — снизьте --voice-level")
        return 1
    print("\nМикрофон слышит. Этот номер и ставьте в --audio-device.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
