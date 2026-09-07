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
#: Убраны «стопэ» — такого слова в словаре модели нет вовсе, она его
#: молча игнорировала, — и обрывок «лини»: на записях он ловился в чужой
#: речи чаще, чем настоящая команда.
COMMANDS = {
    "стоп": ("стоп", "стой", "хватит"),
    "старт": ("старт", "старты", "пуск", "поехали"),
    "линия": ("линия", "линию", "линии"),
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
#: Пороги СНИЖЕНЫ после замеров на записях. Высокий порог оказался не той
#: ручкой: словарь сведён к командам, выбора у распознавателя почти нет, и
#: оценку он ставит высокую даже случайному слову. Порог 0.90 не мешал
#: ложным срабатываниям, зато резал настоящие: сказанное «положение»
#: получало 0.51 и отбрасывалось. От чужой речи теперь защищает счёт слов
#: (см. MAX_WORDS), а порог остался только против совсем невнятного.
CONFIDENCE = {
    "стоп": 0.30,
    "старт": 0.45,
    "линия": 0.45,
    "положение": 0.45,
}
DEFAULT_CONFIDENCE = 0.45

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
    QUEUE_CHUNKS = 16            # звука в запасе, ~2 с при блоке 2000

    #: Во сколько раз пик громче порога тишины, чтобы кусок считался речью.
    PEAK_FACTOR = 4

    #: Сколько кусков после речи всё равно распознавать. Слово почти
    #: всегда лежит на границе кусков, и без этого запаса от него
    #: оставалась половина — модель такое не узнаёт.
    HANGOVER_CHUNKS = 4

    #: Кусок звука. Четверть секунды давала грубую нарезку: короткое
    #: слово рвалось пополам. 125 мс и режет мягче, и команда приходит
    #: вдвое быстрее.
    BLOCK = 2000

    #: Больше этого слов в окончательном разборе — это не команда.
    #:
    #: Словарь распознавателя намеренно сведён к командам, и обычная речь
    #: раскладывается по ним же: разговор превращается в «старт линия
    #: стоп старт». Команду человек говорит отдельно, одним словом, —
    #: любая добавка означает ровно обратное, что говорили не роботу.
    #:
    #: На записях (11 команд, 10 обрывков обычной речи) правило решает
    #: почти всё: при «не больше одного слова» — 11 команд из 11 и одно
    #: ложное срабатывание; при «не больше двух» — те же 11 команд, но уже
    #: пять ложных; прежний порядок без этой проверки давал ложное
    #: срабатывание на ВСЕХ десяти обрывках.
    MAX_WORDS = 1

    #: Через сколько секунд ту же команду можно принять снова.
    #:
    #: Раньше повтор блокировался, пока не услышим что-то ДРУГОЕ. Сказать
    #: «стоп» дважды подряд было нельзя: второй раз проглатывался молча.
    REPEAT_SECONDS = 1.5

    #: Разумная длительность слова-команды, секунды. Разложенная на
    #: команды болтовня даёт куски то по 50 мс, то по две секунды.
    WORD_SECONDS = (0.10, 1.20)

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

        # Очередь ограничена намеренно. Распознавание временами не
        # успевает за реальным временем, и с неограниченной очередью
        # отставание уже не рассасывается: поток жуёт накопленный звук на
        # полном ходу, отнимает процессор у распознавания руки — кадры
        # проседают, — а команда всё равно приходит с опозданием и не
        # нужна. Лучше выбросить самый старый кусок и слушать живой звук.
        self._queue: queue.Queue[bytes] = queue.Queue(maxsize=self.QUEUE_CHUNKS)
        self.dropped = 0
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
        self.reject_reason = ""     # почему отбросили последний разбор
        self.last_rejected = ""     # то же самое, но наружу, для окна
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
        """Тихий ли кусок. Дешёвая проверка до дорогого распознавания.

        Считается ПИК, а не среднее. Кусок длиной в четверть секунды почти
        целиком состоит из тишины даже когда в нём есть слово: «стоп»
        занимает миллисекунд триста, остальное — пауза. Среднее по такому
        куску падает ниже порога, и слово выбрасывалось не дойдя до
        распознавания. Именно поэтому команды «не слышались», а болтовня
        фоном — длинная и ровная — порог как раз перебивала.

        Среднее всё равно считаем: оно показывается в окне как уровень
        звука, и по нему человек подбирает порог.

        >>> import array
        >>> v = object.__new__(VoiceCommands)
        >>> v.silence_level = 300
        >>> тихо = array.array("h", [10] * 4000).tobytes()
        >>> v._is_silence(тихо)
        True
        >>> слово = array.array("h", [10] * 3800 + [9000] * 200).tobytes()
        >>> v._is_silence(слово)          # среднее ~460, но пик громкий
        False
        """
        import array

        samples = array.array("h")
        samples.frombytes(chunk)
        if not samples:
            return True
        self.level = sum(abs(s) for s in samples) / len(samples)
        peak = max(abs(s) for s in samples)
        return peak < self.silence_level * self.PEAK_FACTOR

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
        """Можно ли верить разбору. Проверок три, и они о разном.

        Одной уверенности мало. Словарь сведён к командам, поэтому у
        распознавателя почти нет выбора: он ставит высокую оценку и
        случайному слову, лишь бы оно было похоже. Отсюда и получалось
        наоборот — болтовня проходила, а сказанная команда отсеивалась.

        Поэтому смотрим ещё на две вещи, от оценки не зависящие:

        * сколько всего слов разобрано. Команду говорят отдельно; цепочка
          из пяти «команд» подряд — это разложенный по словарю разговор;
        * сколько длилось само слово. Настоящее слово занимает от десятых
          долей секунды до секунды, обрывок чужой речи — заметно меньше
          или заметно больше.

        >>> v = object.__new__(VoiceCommands); v.bias = 0.0
        >>> v.rejected = 0; v.last_confidence = 0.0; v.reject_reason = ""
        >>> одно = {"result": [{"word": "стоп", "conf": 0.9,
        ...                     "start": 1.0, "end": 1.4}]}
        >>> v._confident(одно, "стоп")
        True
        >>> болтовня = {"result": [{"word": w, "conf": 0.9,
        ...                         "start": i * 0.2, "end": i * 0.2 + 0.15}
        ...                        for i, w in enumerate(
        ...                            ["старт", "линия", "стоп", "старт"])]}
        >>> v._confident(болтовня, "стоп")
        False
        >>> v.reject_reason
        'слов в разборе 4'
        """
        words = result.get("result") or []
        if not words:
            # Разбора по словам нет — проверять нечем. Раньше такой случай
            # проходил без проверки вовсе, и это был прямой путь для
            # ложного срабатывания.
            self.reject_reason = "нет разбора по словам"
            return False

        if len(words) > self.MAX_WORDS:
            self.reject_reason = f"слов в разборе {len(words)}"
            return False

        variants = set(COMMANDS[command])
        spoken = [w for w in words if w.get("word") in variants]
        if not spoken:
            self.reject_reason = "слова команды нет в разборе"
            return False

        best = max(spoken, key=lambda w: w.get("conf", 0.0))
        self.last_confidence = best.get("conf", 0.0)

        low, high = self.WORD_SECONDS
        length = float(best.get("end", 0.0)) - float(best.get("start", 0.0))
        if not low <= length <= high:
            self.reject_reason = f"длительность {length:.2f} с"
            return False

        if self.last_confidence < self.threshold(command):
            self.reject_reason = (f"уверенность {self.last_confidence:.2f} "
                                  f"< {self.threshold(command):.2f}")
            return False

        self.reject_reason = ""
        return True

    # --- поток -----------------------------------------------------------

    def _feed(self, indata, frames, time_info, status) -> None:
        if status:
            log.debug("микрофон: %s", status)
        try:
            self._queue.put_nowait(bytes(indata))
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self.dropped += 1
            try:
                self._queue.put_nowait(bytes(indata))
            except queue.Full:
                pass

    def _worker(self) -> None:
        recognizer = self._vosk.KaldiRecognizer(
            self._model, self.SAMPLE_RATE, self._grammar)
        # Просим разбор по словам с оценками — без него уверенность
        # проверить нечем, и в шуме команды мерещатся.
        recognizer.SetWords(True)
        try:
            with self._sd.RawInputStream(
                    samplerate=self.SAMPLE_RATE, blocksize=self.BLOCK,
                    device=self.device, dtype="int16", channels=1,
                    callback=self._feed):
                self.status = "слушаю"
                log.info("слушаю голос: старт, стоп, линия, положение")
                hangover = 0
                while not self._stop.is_set():
                    try:
                        chunk = self._queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if self._is_silence(chunk):
                        if hangover <= 0:
                            self.skipped += 1
                            continue
                        # Хвост слова уже тише порога, но модели он нужен
                        # целиком, иначе разбора не выйдет.
                        hangover -= 1
                    else:
                        hangover = self.HANGOVER_CHUNKS

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
                    if command and not final:
                        if command not in PARTIAL_ALLOWED:
                            continue    # ждём окончательного результата
                        # Частичный разбор проверить нечем: оценок по
                        # словам в нём нет. Поэтому принимаем его, только
                        # когда в куске ровно одно слово и это останов —
                        # иначе сюда проходила любая чужая речь, в которой
                        # послышалось «стой».
                        if len(text.split()) > 1:
                            continue
                    if command and final and not self._confident(result, command):
                        self.rejected += 1
                        self.last_rejected = f"{command}: {self.reject_reason}"
                        log.info("«%s» отброшено — %s (текст: %r)",
                                 command, self.reject_reason, text)
                        continue

                    import time as _time
                    now = _time.monotonic()
                    fresh = (command != self.last
                             or now - self.last_heard_at > self.REPEAT_SECONDS)
                    if command and fresh:
                        self.last = command
                        self.heard += 1
                        self.last_heard = command
                        self.last_heard_at = now
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
