"""Скачать модели распознавания. Одинаково на Windows, macOS и Linux.

Модели в репозиторий не кладутся: вместе они весят под сотню мегабайт, а
большая модель речи — три с половиной гигабайта. Гит такое не прощает.
Поэтому их забирает этот скрипт, и вызывают его установщики обеих систем.

Средствами самого питона, без curl, wget, unzip и tar: на чистой Windows
ничего этого нет.

    python tools/setup_models.py             всё нужное для работы
    python tools/setup_models.py --большая-модель-речи
    python tools/setup_models.py --проверить  только посмотреть, чего нет
"""

from __future__ import annotations

import argparse
import shutil
import sys
import tempfile
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

КОРЕНЬ = Path(__file__).resolve().parents[1]
ПАПКА = КОРЕНЬ / "models"

ПОЗА = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
        "pose_landmarker_{вид}/float16/1/pose_landmarker_{вид}.task")

#: Что нужно обязательно. Тяжёлую модель позы тоже качаем: без неё нечем
#: проверить, спасает ли она на камере сверху, а это первый же вопрос при
#: настройке.
ФАЙЛЫ = [
    ("pose_landmarker_lite.task", ПОЗА.format(вид="lite"), 6),
    ("pose_landmarker_full.task", ПОЗА.format(вид="full"), 9),
    ("pose_landmarker_heavy.task", ПОЗА.format(вид="heavy"), 30),
    ("hand_landmarker.task",
     "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
     "hand_landmarker/float16/1/hand_landmarker.task", 8),
]

#: Модель речи по умолчанию НЕ качается: 87 МБ ради возможности, без
#: которой программа работает целиком. Ключ --голос забирает её.
АРХИВЫ: list = []

ГОЛОС = ("vosk-ru",
         "https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip",
         "vosk-model-small-ru-0.22", 45)

БОЛЬШАЯ_РЕЧЬ = ("vosk-model-ru-0.42",
                "https://alphacephei.com/vosk/models/vosk-model-ru-0.42.zip",
                "vosk-model-ru-0.42", 1900)


#: Полоска рисуется только в живой терминал. В файл она пишет по строке
#: на каждый прочитанный кусок — сто шестьдесят килобайт мусора на одну
#: загрузку, в котором тонет настоящая ошибка.
ЖИВОЙ_ТЕРМИНАЛ = sys.stdout.isatty()


def показать_ход(готово: int, всего: int, имя: str) -> None:
    """Полоска загрузки в одну строку.

    Без неё скачивание большой модели выглядит как зависший терминал:
    полтора гигабайта идут долго, и человек успевает нажать Ctrl-C.
    """
    if not ЖИВОЙ_ТЕРМИНАЛ:
        return
    if всего <= 0:
        sys.stdout.write(f"\r  {имя}: {готово / 1048576:.0f} МБ")
    else:
        доля = готово / всего
        полоса = "█" * int(доля * 28) + "·" * (28 - int(доля * 28))
        sys.stdout.write(f"\r  {имя}: {полоса} {доля * 100:3.0f}% "
                         f"({готово / 1048576:.0f}/{всего / 1048576:.0f} МБ)")
    sys.stdout.flush()


def скачать(ссылка: str, куда: Path, имя: str) -> None:
    """Забрать файл, показывая ход. Недокачанное не остаётся."""
    временный = куда.with_suffix(куда.suffix + ".часть")
    try:
        with urllib.request.urlopen(ссылка, timeout=60) as ответ:
            всего = int(ответ.headers.get("Content-Length") or 0)
            готово = 0
            with open(временный, "wb") as файл:
                while кусок := ответ.read(1 << 16):
                    файл.write(кусок)
                    готово += len(кусок)
                    показать_ход(готово, всего, имя)
        if ЖИВОЙ_ТЕРМИНАЛ:
            print()
        else:
            print(f"  {имя}: {готово / 1048576:.0f} МБ, скачано")
        # Переименование в самом конце: если скачивание оборвётся, на
        # диске останется .часть, а не файл, который выглядит целым и
        # ломает программу уже при запуске.
        временный.replace(куда)
    except BaseException:
        временный.unlink(missing_ok=True)
        raise


def распаковать(архив: Path, куда: Path, внутренняя: str) -> None:
    """Развернуть zip и переименовать папку внутри в нужное имя."""
    with tempfile.TemporaryDirectory() as врем:
        with zipfile.ZipFile(архив) as z:
            z.extractall(врем)
        источник = Path(врем) / внутренняя
        if not источник.is_dir():
            вложенные = [p for p in Path(врем).iterdir() if p.is_dir()]
            if len(вложенные) != 1:
                raise RuntimeError(f"не понял, что внутри архива: {вложенные}")
            источник = вложенные[0]
        if куда.exists():
            shutil.rmtree(куда)
        shutil.move(str(источник), str(куда))


def чего_нет(с_большой: bool = False, с_голосом: bool = False) -> list[str]:
    """Список того, что ещё не скачано."""
    нет = [имя for имя, _, _ in ФАЙЛЫ if not (ПАПКА / имя).is_file()]
    нет += [имя for имя, _, _, _ in АРХИВЫ if not (ПАПКА / имя).is_dir()]
    if с_голосом and not (ПАПКА / ГОЛОС[0]).is_dir():
        нет.append(ГОЛОС[0])
    if с_большой and not (ПАПКА / БОЛЬШАЯ_РЕЧЬ[0]).is_dir():
        нет.append(БОЛЬШАЯ_РЕЧЬ[0])
    return нет


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--голос", action="store_true",
                   help="ещё и модель распознавания речи (87 МБ). Без неё "
                        "работает всё, кроме голосовых команд")
    p.add_argument("--большая-модель-речи", action="store_true",
                   dest="большая",
                   help="ещё и vosk-model-ru-0.42: 1.9 ГБ архивом и 3.5 ГБ "
                        "на диске. Точнее в шуме, но грузится 16 секунд при "
                        "каждом запуске. Без неё всё работает")
    p.add_argument("--проверить", action="store_true",
                   help="только сказать, чего не хватает")
    args = p.parse_args()

    ПАПКА.mkdir(parents=True, exist_ok=True)

    if args.проверить:
        нет = чего_нет(args.большая, args.голос)
        if нет:
            print("не хватает: " + ", ".join(нет))
            return 1
        print("все модели на месте")
        return 0

    задания = [(имя, ссылка, вес, None, None)
               for имя, ссылка, вес in ФАЙЛЫ]
    задания += [(имя, ссылка, вес, "zip", внутри)
                for имя, ссылка, внутри, вес in АРХИВЫ]
    if args.голос:
        имя, ссылка, внутри, вес = ГОЛОС
        задания.append((имя, ссылка, вес, "zip", внутри))
    if args.большая:
        имя, ссылка, внутри, вес = БОЛЬШАЯ_РЕЧЬ
        задания.append((имя, ссылка, вес, "zip", внутри))

    всего_мб = sum(в for _, _, в, _, _ in задания)
    print(f"нужно скачать до {всего_мб} МБ в {ПАПКА}\n")

    for имя, ссылка, _, вид, внутри in задания:
        цель = ПАПКА / имя
        есть = цель.is_dir() if вид == "zip" else цель.is_file()
        if есть:
            print(f"  {имя}: уже есть")
            continue
        try:
            if вид == "zip":
                архив = ПАПКА / f"{имя}.zip"
                скачать(ссылка, архив, имя)
                print(f"  {имя}: распаковываю...")
                распаковать(архив, цель, внутри)
                архив.unlink(missing_ok=True)
            else:
                скачать(ссылка, цель, имя)
        except (urllib.error.URLError, OSError, RuntimeError) as сбой:
            print(f"\n  {имя}: НЕ СКАЧАЛОСЬ — {сбой}")
            print("  проверьте интернет и запустите этот скрипт снова: "
                  "уже скачанное второй раз не качается")
            return 1

    нет = чего_нет(args.большая, args.голос)
    if нет:
        print("\nвсё ещё не хватает: " + ", ".join(нет))
        return 1
    print("\nмодели на месте")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
