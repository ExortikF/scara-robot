"""Голосовые команды офлайн: старт, стоп, нули.

Распознавание — vosk, целиком на машине, без интернета. Модель лежит в
``models/vosk-ru``.

Зачем именно голос: у оператора обе руки перед камерой, и тянуться к
клавиатуре — значит выйти из кадра ровно в тот момент, когда нужно
остановить движение.

Устройство простое и намеренно тупое. Никакого разбора фраз: в потоке
распознанных слов ищутся ключевые. Так надёжнее — модель маленькая и
целые фразы разбирает плохо, а отдельное слово ловит уверенно.
"""

from __future__ import annotations

import json
import logging
import queue
import threading
from pathlib import Path
from typing import Callable

log = logging.getLogger(__name__)

#: Слова на каждую команду. Их несколько, потому что распознаватель
#: путает похожие, а цена пропущенного «стоп» выше цены лишнего.
COMMANDS = {
    "стоп": ("стоп", "стой", "стопе", "стоб", "хватит"),
    "старт": ("старт", "старты", "пуск", "поехали", "начали"),
    "нули": ("нули", "ноли", "ноль", "нуль", "нолик", "середина"),
}

MODEL_DIR = Path(__file__).resolve().parents[2] / "models" / "vosk-ru"


class VoiceCommands:
    """Слушает микрофон и вызывает обработчик на каждую узнанную команду.

    Работает в отдельном потоке: распознавание занимает десятки
    миллисекунд, и в потоке окна оно подвешивало бы интерфейс — ровно ту
    кнопку «стоп», ради которой всё и делается.
    """

    SAMPLE_RATE = 16000

    def __init__(self, on_command: Callable[[str], None],
                 model_dir: Path | None = None, device: int | None = None):
        try:
            import sounddevice
            import vosk
        except ImportError as exc:
            raise SystemExit(
                "для голосовых команд нужны vosk и sounddevice:\n"
                f"    ./.venv-rs/bin/pip install vosk sounddevice\n"
                f"(не найдено: {exc.name})") from exc

        self._sd = sounddevice
        self._vosk = vosk
        self.on_command = on_command
        self.device = device

        folder = model_dir or MODEL_DIR
        if not folder.is_dir():
            raise SystemExit(
                f"нет модели распознавания: {folder}\n\n"
                "Скачайте один раз (около 45 МБ):\n"
                "    curl -L -o /tmp/vosk-ru.zip \\\n"
                "        https://alphacephei.com/vosk/models/"
                "vosk-model-small-ru-0.22.zip\n"
                "    unzip -q /tmp/vosk-ru.zip -d models/\n"
                "    mv models/vosk-model-small-ru-0.22 models/vosk-ru\n")

        vosk.SetLogLevel(-1)          # иначе засыпает терминал отладкой
        self._model = vosk.Model(str(folder))

        # Ограничиваем словарь: модель тогда не пытается расслышать в шуме
        # посторонние слова и почти не выдаёт ложных срабатываний.
        vocabulary = sorted({w for words in COMMANDS.values() for w in words})
        self._grammar = json.dumps(vocabulary + ["[unk]"], ensure_ascii=False)

        self._queue: queue.Queue[bytes] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.heard = 0
        self.last = ""

    # --- разбор ----------------------------------------------------------

    @staticmethod
    def match(text: str) -> str | None:
        """Найти команду в распознанном тексте. None — ничего знакомого.

        >>> VoiceCommands.match("стоп")
        'стоп'
        >>> VoiceCommands.match("а ну ка стой")
        'стоп'
        >>> VoiceCommands.match("поехали давай")
        'старт'
        >>> VoiceCommands.match("погода хорошая") is None
        True
        """
        words = set(text.lower().split())
        # «стоп» проверяем первой: если в фразе есть и она, и другая
        # команда, останов должен побеждать.
        for name in ("стоп", "старт", "нули"):
            if words & set(COMMANDS[name]):
                return name
        return None

    # --- поток -----------------------------------------------------------

    def _feed(self, indata, frames, time_info, status) -> None:
        if status:
            log.debug("микрофон: %s", status)
        self._queue.put(bytes(indata))

    def _worker(self) -> None:
        recognizer = self._vosk.KaldiRecognizer(
            self._model, self.SAMPLE_RATE, self._grammar)
        try:
            with self._sd.RawInputStream(
                    samplerate=self.SAMPLE_RATE, blocksize=4000,
                    device=self.device, dtype="int16", channels=1,
                    callback=self._feed):
                log.info("слушаю голос: старт, стоп, нули")
                while not self._stop.is_set():
                    try:
                        chunk = self._queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if recognizer.AcceptWaveform(chunk):
                        text = json.loads(recognizer.Result()).get("text", "")
                    else:
                        # Частичный результат: команду отдаём, не дожидаясь
                        # паузы в речи. Для «стоп» это заметно быстрее.
                        text = json.loads(
                            recognizer.PartialResult()).get("partial", "")
                    if not text:
                        continue
                    command = self.match(text)
                    if command and command != self.last:
                        self.last = command
                        self.heard += 1
                        log.info("услышано: %s", command)
                        try:
                            self.on_command(command)
                        except Exception:
                            log.exception("обработчик команды упал")
                    elif not command:
                        self.last = ""      # готовы услышать ту же снова
        except Exception as exc:
            log.error("микрофон недоступен: %s", exc)

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
