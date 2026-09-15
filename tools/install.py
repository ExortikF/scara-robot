"""Поставить всё, что нужно для работы. Windows, macOS, Linux.

Запускается СИСТЕМНЫМ питоном, а не из окружения — окружение он и создаёт.
Обычно вызывается из `установить.bat` или `установить.command`, но можно
и напрямую:

    python tools/install.py
    python tools/install.py --большая-модель-речи

Что делает по шагам:

1. проверяет версию питона;
2. создаёт окружение `.venv` рядом с проектом;
3. ставит библиотеки из `requirements.txt`;
4. скачивает модели распознавания;
5. проверяет, что всё вместе действительно работает.

Последний шаг важнее, чем кажется. «Установилось без ошибок» и «работает»
— разные вещи: библиотека может встать и упасть при первом же импорте
из-за несовпадения версий. Лучше узнать это сейчас, а не при первом
запуске у станка.
"""

from __future__ import annotations

import argparse
import os
import platform
import subprocess
import sys
from pathlib import Path

КОРЕНЬ = Path(__file__).resolve().parents[1]
ОКРУЖЕНИЕ = КОРЕНЬ / ".venv"

WINDOWS = sys.platform.startswith("win")

#: Границы проверены по готовым пакетам, а не взяты на глаз:
#: pyrealsense2 собран под 3.9-3.14, numpy под 3.12 и новее, mediapipe и
#: vosk от версии питона не зависят вовсе. Узкое место — pyrealsense2:
#: если готового пакета нет, его пришлось бы собирать из исходников, а
#: это вечер работы и выходит не у всех.
МИН_ПИТОН = (3, 10)
МАКС_ПИТОН = (3, 14)


def питон_окружения() -> Path:
    """Где в созданном окружении лежит интерпретатор."""
    return ОКРУЖЕНИЕ / ("Scripts/python.exe" if WINDOWS else "bin/python")


def шаг(номер: int, всего: int, текст: str) -> None:
    print(f"\n[{номер}/{всего}] {текст}")


def выполнить(команда: list[str], что: str) -> bool:
    """Запустить и, если не вышло, показать причину, а не просто код."""
    готово = subprocess.run(команда, capture_output=True, text=True)
    if готово.returncode == 0:
        return True
    print(f"  НЕ ВЫШЛО: {что}")
    хвост = (готово.stderr or готово.stdout or "").strip().splitlines()
    for строка in хвост[-12:]:
        print(f"    {строка}")
    return False


def есть_tkinter() -> bool:
    """Собран ли этот питон с поддержкой окон."""
    try:
        import tkinter  # noqa: F401
    except Exception:
        return False
    return True


def проверить_питон() -> bool:
    версия = sys.version_info[:2]
    разрядность = 64 if sys.maxsize > 2 ** 32 else 32
    print(f"  питон {platform.python_version()} "
          f"({platform.system()} {platform.machine()}, {разрядность} бит)")

    # 32 бита — самая частая беда на Windows, и по ошибке pip её не
    # опознать: он пишет «no matching distribution found for numpy», как
    # будто пропал интернет. На деле колёс под 32-битную Windows не
    # выпускают уже несколько лет ни numpy, ни mediapipe, ни opencv.
    # С сайта питона по умолчанию нередко качается именно 32-битный
    # установщик, поэтому попасть сюда проще, чем кажется.
    if разрядность == 32:
        print("  ЭТО 32-БИТНЫЙ ПИТОН — нужных пакетов под него не бывает.")
        print("  Ни numpy, ни mediapipe, ни opencv давно не выпускают "
              "32-битных сборок,")
        print("  и установка обрывается на «no matching distribution "
              "found for numpy».")
        print()
        print("  Поставьте 64-битный: на python.org это файл с пометкой")
        print("  «Windows installer (64-bit)» — обычный «Windows installer»")
        print("  без пометки бывает 32-битным.")
        return False
    if версия < МИН_ПИТОН:
        print(f"  СЛИШКОМ СТАРЫЙ. Нужен хотя бы "
              f"{МИН_ПИТОН[0]}.{МИН_ПИТОН[1]}.")
        print("  Скачать: https://www.python.org/downloads/")
        return False
    if версия > МАКС_ПИТОН:
        print(f"  СЛИШКОМ НОВЫЙ. Проверено до "
              f"{МАКС_ПИТОН[0]}.{МАКС_ПИТОН[1]} включительно.")
        print("  Под свежую версию питона пакеты выходят не сразу, и "
              "установка обрывается")
        print("  на «no matching distribution found» — это не про "
              "интернет, а про то,")
        print("  что колеса под неё ещё нет.")
        print("  Под свежие версии ещё не собраны готовые пакеты для "
              "камеры RealSense,")
        print(f"  и ставить их пришлось бы из исходников. Возьмите "
              f"{МАКС_ПИТОН[0]}.{МАКС_ПИТОН[1]}.")
        return False

    # Окно управления — это tkinter, и без него смысла ставить остальное
    # нет. Проверяем СЕЙЧАС, а не после пяти минут установки: питон
    # бывает собран без поддержки окон, и на macOS это обычное дело.
    if not есть_tkinter():
        print("  В ЭТОМ ПИТОНЕ НЕТ TKINTER — окно управления не откроется.")
        if sys.platform == "darwin":
            print("  Поставьте:  brew install python-tk@"
                  f"{версия[0]}.{версия[1]}")
            print("  или возьмите питон с python.org — там Tk уже внутри.")
        elif WINDOWS:
            print("  Переустановите питон с python.org и не снимайте "
                  "галочку\n  «tcl/tk and IDLE».")
        else:
            print("  Поставьте пакет python3-tk из репозитория системы.")
        return False
    return True


def создать_окружение() -> bool:
    if питон_окружения().is_file():
        print(f"  окружение уже есть: {ОКРУЖЕНИЕ}")
        return True
    print(f"  создаю {ОКРУЖЕНИЕ}")
    if not выполнить([sys.executable, "-m", "venv", str(ОКРУЖЕНИЕ)],
                     "не создалось окружение"):
        return False
    return питон_окружения().is_file()


def поставить_библиотеки() -> bool:
    питон = str(питон_окружения())
    print("  обновляю pip...")
    выполнить([питон, "-m", "pip", "install", "--quiet", "--upgrade", "pip"],
              "pip не обновился (не страшно)")

    print("  ставлю библиотеки, это несколько минут...")
    if not выполнить([питон, "-m", "pip", "install", "--quiet",
                      "-r", str(КОРЕНЬ / "requirements.txt")],
                     "библиотеки не встали"):
        return False

    # Камера ставится отдельно и НЕ обязательна: без неё работает всё,
    # кроме RealSense, — обычная веб-камера и симулятор доступны. На
    # системах без готового колеса ставить её из исходников ради этого
    # незачем, поэтому сбой здесь не валит установку.
    print("  ставлю поддержку камеры RealSense...")
    if выполнить([питон, "-m", "pip", "install", "--quiet", "pyrealsense2"],
                 "pyrealsense2 не встал"):
        print("    камера RealSense: готова")
    elif sys.platform == "darwin":
        print("    камера RealSense: готовых пакетов под macOS нет вообще,")
        print("    их собирают из исходников. Обычная веб-камера и "
              "симулятор работают.")
    else:
        print("    камера RealSense: НЕ встала. Всё остальное работает;")
        print("    попробуйте питон другой версии.")
    return True


def скачать_модели(большая: bool) -> bool:
    команда = [str(питон_окружения()), str(КОРЕНЬ / "tools" / "setup_models.py")]
    if большая:
        команда.append("--большая-модель-речи")
    # Без capture: у загрузки своя полоска, и прятать её незачем.
    return subprocess.run(команда).returncode == 0


def проверить_работу() -> bool:
    """Импортировать всё и собрать окно с заглушкой вместо камеры."""
    питон = str(питон_окружения())

    проба = (
        "import sys; sys.path.insert(0, r'%s')\n"
        "import cv2, numpy, serial, pymodbus, mediapipe\n"
        "import tkinter, PIL.Image, PIL.ImageTk\n"
        "from scara_drive.follow import MultiAxisFollower\n"
        "from scara_drive.armtrack import angles_from_points\n"
        "print('импорты: в порядке')\n"
    ) % (КОРЕНЬ / "src")
    if not выполнить([питон, "-c", проба], "библиотеки не импортируются"):
        return False
    print("  импорты: в порядке")

    print("  собираю окно управления...")
    if not выполнить([питон, str(КОРЕНЬ / "tools" / "arm_control.py"),
                      "--smoke", "2"],
                     "окно не собралось"):
        return False
    print("  окно: собирается и закрывается")
    return True


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--большая-модель-речи", action="store_true",
                   dest="большая",
                   help="скачать ещё и большую модель распознавания речи "
                        "(1.9 ГБ). Без неё всё работает")
    p.add_argument("--без-проверки", action="store_true",
                   help="не проверять работу после установки")
    args = p.parse_args()

    print("=" * 62)
    print("  Установка: SCARA, повторяющий движения руки")
    print("=" * 62)

    всего = 4 if args.без_проверки else 5

    шаг(1, всего, "Проверяю питон")
    if not проверить_питон():
        return 1

    шаг(2, всего, "Создаю окружение")
    if not создать_окружение():
        return 1

    шаг(3, всего, "Ставлю библиотеки")
    if not поставить_библиотеки():
        return 1

    шаг(4, всего, "Скачиваю модели распознавания")
    if not скачать_модели(args.большая):
        return 1

    if not args.без_проверки:
        шаг(5, всего, "Проверяю, что всё работает")
        if not проверить_работу():
            print("\nУстановилось, но проверка не прошла. Смотрите выше.")
            return 1

    запуск = ("запустить.bat" if WINDOWS else "./запустить.command")
    print("\n" + "=" * 62)
    print("  Готово.")
    print("=" * 62)
    print(f"\nЗапуск без железа, чтобы посмотреть:  {запуск} --simulate")
    print(f"Запуск с приводами и камерой:         {запуск}")
    print("\nВсе команды с пояснениями — в файле КОМАНДЫ.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
