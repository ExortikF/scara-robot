"""Различия между Windows, macOS и Linux — все в одном месте.

Различий немного, но они разбросаны по неожиданным местам: имя
последовательного порта, чем проиграть звук, кто держит камеру. Если
раскидать проверки системы по коду, каждая новая правка будет забывать
про одну из систем. Здесь они собраны, и остальной код спрашивает
отсюда, а не у ``sys.platform``.

Правило простое: **на чужой системе всё должно не мешать, а не падать.**
Демон камеры, которого на Windows нет, — это не ошибка, это отсутствие
работы.
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

log = logging.getLogger(__name__)

WINDOWS = sys.platform.startswith("win")
MACOS = sys.platform == "darwin"
LINUX = sys.platform.startswith("linux")

#: Как выглядит порт переходника USB<->RS-485 в этой системе. Идёт в
#: подсказки и сообщения об ошибках: «укажите /dev/cu.usbserial-XXXX» на
#: Windows сбивает с толку, там таких имён не бывает.
ПРИМЕР_ПОРТА = "COM3" if WINDOWS else "/dev/cu.usbserial-XXXX"

#: Нужны ли особые права, чтобы библиотека дотянулась до камеры.
#: macOS не отдаёт RealSense без root. Windows отдаёт как есть.
КАМЕРЕ_НУЖЕН_ROOT = MACOS


#: Как выглядят пути к питону и pip внутри окружения. Идут в подсказки:
#: «поставьте через .venv/bin/pip» на Windows никуда не ведёт.
ПИТОН_ОКРУЖЕНИЯ = (r".venv\Scripts\python.exe" if WINDOWS
                   else ".venv/bin/python")
PIP_ОКРУЖЕНИЯ = (r".venv\Scripts\pip.exe" if WINDOWS else ".venv/bin/pip")


def имя_системы() -> str:
    return "Windows" if WINDOWS else "macOS" if MACOS else "Linux"


def запущен_от_root() -> bool:
    """Запущены ли мы с правами администратора.

    На Windows ``os.geteuid`` не существует вовсе — обращение к нему
    роняет программу на AttributeError. Там понятия «root» в этом смысле
    нет, и ответ «нет» верен: особых прав для камеры и порта не требуется.
    """
    import os

    if WINDOWS:
        return False
    return os.geteuid() == 0


def порт_существует(порт: str) -> bool:
    """Есть ли такой последовательный порт в системе.

    На macOS и Linux порт — файл в /dev, наличие проверяется напрямую.
    На Windows порт называется ``COM3`` и файлом НЕ является:
    ``os.path.exists("COM3")`` там всегда ложь. Проверять так значит
    объявлять исправный переходник пропавшим при каждом открытии — то
    есть сломать связь целиком. Поэтому там спрашиваем список у системы.
    """
    import os

    if not WINDOWS:
        return os.path.exists(порт)

    from serial.tools import list_ports as _lp

    return any(item.device.upper() == порт.upper() for item in _lp.comports())


def освободить_камеру(пауза: float = 0.6) -> None:
    """Согнать системного демона, который держит камеру.

    Только macOS: там VDCAssistant захватывает RGB-интерфейс, как только
    камерой интересуется что-нибудь через AVFoundation — включая нашу же
    неудавшуюся попытку. Пока он держит интерфейс, librealsense отвечает
    «failed to set power state». Убивать безопасно: демон поднимается сам.

    На Windows и Linux такого посредника нет, и делать тут нечего.
    """
    if not MACOS:
        return

    import subprocess
    import time

    subprocess.run(["killall", "VDCAssistant"],
                   capture_output=True, check=False)
    time.sleep(пауза)


def проиграть_звук(путь: Path) -> None:
    """Проиграть WAV, не блокируя вызывающего."""
    import subprocess

    if WINDOWS:
        # winsound входит в стандартную поставку Windows-питона,
        # внешних проигрывателей там может не быть вовсе.
        import winsound

        winsound.PlaySound(str(путь),
                           winsound.SND_FILENAME | winsound.SND_ASYNC)
        return

    команда = ["afplay"] if MACOS else ["aplay", "-q"]
    try:
        subprocess.Popen([*команда, str(путь)],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        log.warning("нечем проиграть звук: нет %s", команда[0])


def наговорить(текст: str, куда: Path) -> bool:
    """Записать фразу голосом системы в WAV 16 кГц моно.

    Нужно для проверки распознавания: живой микрофон для замеров не
    годится, результат зависит от того, как человек сказал, и повторить
    вчерашний замер нельзя. Системный голос произносит одинаково.

    Голоса у систем разные, поэтому числа между Windows и macOS впрямую
    не сравнивают — сравнивают версии программы на одной машине.

    Возвращает False, если синтез в этой системе недоступен.
    """
    import subprocess

    if MACOS:
        сырое = куда.with_suffix(".aiff")
        try:
            subprocess.run(["say", "-v", "Milena", "-o", str(сырое), текст],
                           check=True, capture_output=True)
            subprocess.run(["afconvert", "-f", "WAVE", "-d", "LEI16@16000",
                            "-c", "1", str(сырое), str(куда)],
                           check=True, capture_output=True)
        except (FileNotFoundError, subprocess.CalledProcessError) as сбой:
            log.warning("синтез речи не вышел: %s", сбой)
            return False
        finally:
            сырое.unlink(missing_ok=True)
        return True

    if WINDOWS:
        # SAPI есть в любой Windows. Он пишет WAV сразу, но в формате
        # звуковой карты, поэтому частоту задаём явно: распознаватель
        # ждёт ровно 16 кГц моно и на другом молча выдаёт чепуху.
        скрипт = (
            "Add-Type -AssemblyName System.Speech;"
            "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer;"
            "$f = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo("
            "16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,"
            " [System.Speech.AudioFormat.AudioChannel]::Mono);"
            f"$s.SetOutputToWaveFile('{куда}', $f);"
            f"$s.Speak('{текст}');"
            "$s.Dispose();"
        )
        try:
            subprocess.run(["powershell", "-NoProfile", "-Command", скрипт],
                           check=True, capture_output=True)
        except (FileNotFoundError, subprocess.CalledProcessError) as сбой:
            log.warning("синтез речи не вышел: %s", сбой)
            return False
        return куда.is_file()

    log.warning("на %s синтез речи не настроен — проверить нечем",
                имя_системы())
    return False
