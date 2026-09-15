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

МОДЕЛИ = Path(__file__).resolve().parents[2] / "models"

#: Большая модель, если она скачана, и маленькая иначе. Порядок важен:
#: первая найденная и берётся.
ПОРЯДОК_МОДЕЛЕЙ = ("vosk-model-ru-0.42", "vosk-ru")


def выбрать_модель() -> Path:
    """Какая модель распознавания будет использована.

    Большая точнее на шумном фоне, и раз человек её скачал — значит она
    ему нужна, спрашивать об этом каждый запуск незачем. Если её нет,
    работает маленькая, и программа от этого не ломается: разница между
    ними на замерах невелика.

    Платится за большую только временем запуска: она разворачивается
    около двадцати секунд. Поэтому грузится она в рабочем потоке, а окно
    в это время уже открыто и кнопки нажимаются.

    >>> выбрать_модель().parent.name
    'models'
    """
    for имя in ПОРЯДОК_МОДЕЛЕЙ:
        папка = МОДЕЛИ / имя
        if папка.is_dir():
            return папка
    return МОДЕЛИ / ПОРЯДОК_МОДЕЛЕЙ[-1]      # чтобы было что показать в ошибке


#: Оставлено ради старого кода, который берёт путь отсюда.
MODEL_DIR = выбрать_модель()


from .platform_bits import ПИТОН_ОКРУЖЕНИЯ, PIP_ОКРУЖЕНИЯ


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

    #: Сколько первых кусков уходит на замер фона.
    #:
    #: Порог тишины подобрать заранее нельзя: он зависит от микрофона,
    #: расстояния и от того, что шумит рядом. На тележке с силовой частью
    #: фон совсем не тот, что на столе. Поэтому первые полторы секунды
    #: программа молча слушает и ставит порог ВЫШЕ измеренного фона —
    #: иначе распознавание будет молотить шум и отбирать процессор у
    #: камеры, а команды в этом шуме тонуть.
    CALIBRATE_CHUNKS = 12

    #: Во сколько раз порог выше измеренного фона.
    CALIBRATE_FACTOR = 2.5

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
                "голосовые команды не входят в базовую установку.\n\n"
                "Чтобы включить, две команды из папки проекта:\n"
                f"    {PIP_ОКРУЖЕНИЯ} install vosk sounddevice\n"
                f"    {ПИТОН_ОКРУЖЕНИЯ} tools/setup_models.py --голос\n\n"
                f"(не найдено: {exc.name})") from exc

        self._sd = sounddevice
        self._vosk = vosk
        self.on_command = on_command
        self.device = device
        self.silence_level = (self.SILENCE_LEVEL if silence_level is None
                              else silence_level)
        self.skipped = 0            # сколько кусков пропущено как тишина

        # Спрашиваем каждый раз, а не берём MODEL_DIR: модель могли
        # скачать уже после запуска, да и при проверках папка подменяется.
        folder = model_dir or выбрать_модель()
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

        # Модель грузится в РАБОЧЕМ потоке, а не здесь. Большая модель
        # разворачивается двадцать секунд, а этот конструктор вызывают из
        # потока окна: загрузка тут означала бы замерший интерфейс на всё
        # это время — и первое, что человек попробовал бы нажать, была бы
        # кнопка «стоп», которая тоже не нажалась бы.
        self._folder = folder
        self._model = None

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
        self.muted = False          # микрофон заглушён человеком
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

    @classmethod
    def level_for(cls, фон: list[float]) -> float:
        """Порог тишины по замеренному фону.

        Берётся не среднее, а близкое к максимуму: провалы между стуками
        тележки — тоже фон, и равняться на них значит принимать каждый
        стук за речь.

        >>> VoiceCommands.level_for([10.0] * 12)      # тихая комната
        150.0
        >>> VoiceCommands.level_for([400.0] * 12)     # шумно
        1000.0
        """
        фон = sorted(фон)
        верх = фон[int(len(фон) * 0.8)]
        return max(cls.SILENCE_LEVEL / 2, верх * cls.CALIBRATE_FACTOR)

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
        if self._model is None:
            self.status = "загружаю модель"
            начало = __import__("time").monotonic()
            try:
                self._model = self._vosk.Model(str(self._folder))
            except Exception as сбой:
                self.status = "нет модели"
                self.error = str(сбой)
                log.error("модель распознавания не загрузилась: %s", сбой)
                return
            ушло = __import__("time").monotonic() - начало
            log.info("модель речи %s загружена за %.1f с",
                     self._folder.name, ушло)

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
                заглушены = False
                фон: list[float] = []
                while not self._stop.is_set():
                    try:
                        chunk = self._queue.get(timeout=0.2)
                    except queue.Empty:
                        continue

                    if self.muted:
                        # Заглушено — кусок даже не распознаём. Просто не
                        # слушать команды мало: распознавание всё равно
                        # молотило бы процессор впустую, а ради этого
                        # заглушку и включают, когда рядом говорят.
                        self.status = "заглушен"
                        заглушены = True
                        continue
                    if заглушены:
                        # После паузы у распознавателя внутри остаётся
                        # обрывок фразы, сказанной до заглушки. Без сброса
                        # он склеится с первым же новым словом.
                        recognizer.Reset()
                        заглушены = False
                        hangover = 0
                        self.status = "слушаю"

                    if len(фон) < self.CALIBRATE_CHUNKS:
                        self._is_silence(chunk)     # заполняет self.level
                        фон.append(self.level)
                        if len(фон) == self.CALIBRATE_CHUNKS:
                            self.silence_level = self.level_for(фон)
                            self.status = "слушаю"
                            log.info("фон %.0f, порог тишины %.0f",
                                     sum(фон) / len(фон), self.silence_level)
                        else:
                            self.status = "слушаю фон"
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
