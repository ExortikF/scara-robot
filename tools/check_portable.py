"""Найти в коде то, что работает только на одной системе.

Проверить программу на Windows с мака нельзя, а сломать её оттуда —
запросто, и обычно молча: `os.geteuid` на Windows не существует вовсе,
`killall` не находится, а `os.path.exists("COM3")` всегда ложь, из-за чего
исправный переходник объявляется пропавшим при каждом открытии.

Такие вещи не ловятся ни компиляцией, ни доктестами на маке — они ловятся
только на Windows, то есть у того, кому проект отдали. Поэтому здесь
простой обыск по словам.

Правило: системные различия живут в ``scara_drive/platform_bits.py`` и
больше нигде. Если что-то нужно делать по-разному — заводите там функцию,
а не проверку системы по месту.

    python tools/check_portable.py
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

КОРЕНЬ = Path(__file__).resolve().parents[1]

#: Где различиям систем быть МОЖНО.
РАЗРЕШЕНО = {
    "src/scara_drive/platform_bits.py",
    "tools/install.py",
    "tools/setup_models.py",
    "tools/check_portable.py",
}

#: Что ищем и чем это плохо.
ЗАПРЕТЫ = [
    (r"\bos\.gete?uid\b",
     "на Windows такой функции нет — программа упадёт на AttributeError"),
    (r"\bos\.getgid\b",
     "на Windows такой функции нет"),
    (r"\"killall\"|'killall'",
     "команды killall на Windows нет"),
    (r"\"afplay\"|'afplay'|\"afconvert\"|'afconvert'",
     "это программы macOS"),
    (r"\bsubprocess\.[a-z]+\(\s*\[\s*[\"']say[\"']",
     "say — синтез речи macOS"),
    (r"os\.path\.exists\(\s*port\b|os\.path\.exists\(\s*args\.port\b",
     "порт на Windows называется COM3 и файлом не является — "
     "проверка всегда ложна"),
    (r"\bsys\.platform\b",
     "проверку системы держим в platform_bits, а не по месту"),
]

#: Подсказки и примеры в тексте — не работающий код. Строку, где путь
#: только упоминается, ругать не за что.
ТЕКСТ = re.compile(r'^\s*(#|>>>|\.\.\.)|help=|"""')


def обыскать() -> list[tuple[str, int, str, str]]:
    находки = []
    for папка in ("src", "tools", "examples"):
        for файл in sorted((КОРЕНЬ / папка).rglob("*.py")):
            относительный = файл.relative_to(КОРЕНЬ).as_posix()
            if относительный in РАЗРЕШЕНО:
                continue
            for номер, строка in enumerate(
                    файл.read_text(encoding="utf-8").splitlines(), 1):
                if ТЕКСТ.search(строка):
                    continue
                for образец, беда in ЗАПРЕТЫ:
                    if re.search(образец, строка):
                        находки.append((относительный, номер,
                                        строка.strip()[:70], беда))
    return находки


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.parse_args()

    находки = обыскать()
    if not находки:
        print("переносимость: различия систем только в platform_bits")
        return 0

    print(f"переносимость: {len(находки)} мест работают не везде\n")
    for файл, номер, строка, беда in находки:
        print(f"  {файл}:{номер}")
        print(f"    {строка}")
        print(f"    -> {беда}\n")
    print("Различия систем выносите в src/scara_drive/platform_bits.py")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
