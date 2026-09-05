"""Голосовые команды офлайн: старт, стоп, линия, положение.

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
    "стоп": ("стоп", "стой", "стопэ", "хватит"),
    "старт": ("старт", "старты", "пуск", "поехали"),
    "линия": ("линия", "линию", "линии", "лини"),
    "положение": ("положение", "положения", "исходное", "домой"),
}

#: Порядок разбора: если в одной фразе услышаны две команды, побеждать
#: должен останов.
PRIORITY = ("стоп", "старт", "линия", "положение")

#: Порог уверенности СВОЙ у каждой команды: цена ошибки у них разная.
#:
#: Ложный «стоп» ничего не портит — робот просто встанет, и человек
#: скажет «старт». Поэтому порог низкий: лучше лишний раз остановиться,
#: чем не расслышать команду, когда она нужна.
#:
#: Ложный «старт» трогает робота с места, когда его никто не звал.
#: Порог высокий: пропустить команду не страшно, повторить её человек
#: успеет, а неожиданное движение — уже происшествие.
#:
#: Ложная «линия» переопределяет середину и портит настройку — тоже
#: неприятно, но не опасно.
CONFIDENCE = {
    "стоп": 0.35,
    "старт": 0.90,
    "линия": 0.80,
    # Возврат в исходное — движение по всей рабочей зоне, и начинается
    # оно без участия человека. Порог как у пуска.
    "положение": 0.90,
}
DEFAULT_CONFIDENCE = 0.75

#: На частичном результате принимается только останов. Частичные
#: неустойчивы — слово может смениться следующим куском, — но ждать
#: паузы в речи ради «стоп» неправильно: он нужен немедленно.
PARTIAL_ALLOWED = ("стоп",)

MODEL_DIR = Path(__file__).resolve().parents[2] / "models" / "vosk-ru"


class VoiceCommands:
    """Слушает микрофон и вызывает обработчик на каждую узнанную команду.

    Работает в отдельном потоке: распознавание занимает десятки
    миллисекунд, и в потоке окна оно подвешивало бы интерфейс — ровно ту
    кнопку «стоп», ради которой всё и делается.
    """

    SAMPLE_RATE = 16000

    #: Ниже этого уровня кусок считается тишиной и не распознаётся.
    #:
    #: Распознавание — не бесплатная операция, а тишина занимает почти всё
    #: время. Без этой отсечки оно молотит вхолостую и отбирает процессор
    #: у камеры: кадров становится заметно меньше.
    SILENCE_LEVEL = 300          # среднее по модулю, шкала int16

    def __init__(self, on_command: Callable[[str], None],
                 model_dir: Path | None = None, device: int | None = 0,
                 silence_level: int | None = None):
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
        self.silence_level = (self.SILENCE_LEVEL if silence_level is None
                              else silence_level)
        self.skipped = 0            # сколько кусков пропущено как тишина

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

        # Состояние наружу: без него «не работает» неотличимо от «не
        # услышал», а под sudo микрофон отказывает молча — macOS выдаёт
        # доступ приложению, и от имени root запрос уходит мимо него.
        self.status = "запуск"
        self.error: str | None = None
        self.last_heard: str | None = None
        self.last_heard_at: float = 0.0
        self.level = 0.0            # текущий уровень звука
        self.rejected = 0           # отброшено по слабой уверенности
        self.last_confidence = 0.0
        self.bias = 0.0             # общий сдвиг всех порогов

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
        >>> VoiceCommands.match("проведи линию")
        'линия'
        >>> VoiceCommands.match("погода хорошая") is None
        True
        """
        words = set(text.lower().split())
        for name in PRIORITY:
            if words & set(COMMANDS[name]):
                return name
        return None

    def _is_silence(self, chunk: bytes) -> bool:
        """Тихий ли кусок. Дешёвая проверка до дорогого распознавания."""
        import array

        samples = array.array("h")
        samples.frombytes(chunk)
        if not samples:
            return True
        level = sum(abs(s) for s in samples) / len(samples)
        self.level = level
        return level < self.silence_level

    def threshold(self, command: str) -> float:
        """Порог для конкретной команды с учётом общей поправки.

        ``bias`` сдвигает все пороги разом: если в вашей мастерской шумно
        или наоборот тихо, подкручивать удобнее одной ручкой, не ломая
        соотношение между командами.

        >>> v = object.__new__(VoiceCommands); v.bias = 0.0
        >>> v.threshold("стоп") < v.threshold("старт")
        True
        """
        base = CONFIDENCE.get(command, DEFAULT_CONFIDENCE)
        return min(0.99, max(0.05, base + self.bias))

    def _confident(self, result: dict, command: str) -> bool:
        """Достаточно ли уверенно распознано слово команды."""
        words = result.get("result") or []
        variants = set(COMMANDS[command])
        scores = [w.get("conf", 1.0) for w in words
                  if w.get("word") in variants]
        if not scores:
            return True          # разбора по словам нет — верим тексту
        best = max(scores)
        self.last_confidence = best
        return best >= self.threshold(command)

    # --- поток -----------------------------------------------------------

    def _feed(self, indata, frames, time_info, status) -> None:
        if status:
            log.debug("микрофон: %s", status)
        self._queue.put(bytes(indata))

    def _worker(self) -> None:
        recognizer = self._vosk.KaldiRecognizer(
            self._model, self.SAMPLE_RATE, self._grammar)
        # Просим разбор по словам с оценками — без него уверенность
        # проверить нечем, и в шуме команды мерещатся.
        recognizer.SetWords(True)
        try:
            with self._sd.RawInputStream(
                    samplerate=self.SAMPLE_RATE, blocksize=4000,
                    device=self.device, dtype="int16", channels=1,
                    callback=self._feed):
                self.status = "слушаю"
                log.info("слушаю голос: старт, стоп, линия, положение")
                while not self._stop.is_set():
                    try:
                        chunk = self._queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if self._is_silence(chunk):
                        self.skipped += 1
                        continue

                    if recognizer.AcceptWaveform(chunk):
                        result = json.loads(recognizer.Result())
                        text = result.get("text", "")
                        final = True
                    else:
                        result = json.loads(recognizer.PartialResult())
                        text = result.get("partial", "")
                        final = False
                    if not text:
                        continue

                    command = self.match(text)
                    if command and not final and command not in PARTIAL_ALLOWED:
                        continue        # ждём окончательного результата
                    if command and final and not self._confident(result, command):
                        self.rejected += 1
                        log.info("«%s» отброшено: уверенность %.2f, нужно "
                                 "%.2f", command, self.last_confidence,
                                 self.threshold(command))
                        continue

                    if command and command != self.last:
                        self.last = command
                        self.heard += 1
                        import time as _time
                        self.last_heard = command
                        self.last_heard_at = _time.monotonic()
                        log.info("услышано: %s", command)
                        try:
                            self.on_command(command)
                        except Exception:
                            log.exception("обработчик команды упал")
                    elif not command:
                        self.last = ""      # готовы услышать ту же снова
        except Exception as exc:
            self.status = "нет доступа"
            self.error = str(exc)
            log.error("микрофон недоступен: %s", exc)
            log.error("под sudo macOS часто не отдаёт микрофон: доступ "
                      "выдаётся приложению, а не root. Проверить: "
                      "python tools/mics.py --listen 5")

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
