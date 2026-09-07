"""Слежение оси за внешней целью в реальном времени.

Цель приходит из источника (мышь, камера) 20-30 раз в секунду, ось должна за
ней идти. От позиционирования «встать в точку» это отличается тем, что цель
непрерывно меняется, поэтому интегральной составляющей здесь нет: она копила
бы историю, которая уже неактуальна, и добавляла бы перерегулирование.

Контур пропорциональный, поверх скоростного режима привода. Ограничения те же,
что и у любого супервизорного контура на RS-485: проход занимает десятки
миллисекунд, привод принимает целые об/мин.
"""

from __future__ import annotations

import logging
import math
import queue
import threading
import time
from dataclasses import dataclass, field

from . import registers as reg

log = logging.getLogger(__name__)


class DriveFault(RuntimeError):
    """Ось остановлена защитой. Продолжать нельзя."""


def shortest_angle_error(target_deg: float, actual_deg: float) -> float:
    """Кратчайшая знаковая ошибка по окружности, в диапазоне [-180, 180).

    >>> shortest_angle_error(1, 359)
    2.0
    >>> shortest_angle_error(359, 1)
    -2.0
    >>> shortest_angle_error(180, 0)
    -180.0
    """
    return ((float(target_deg) - float(actual_deg) + 180.0) % 360.0) - 180.0


def normalize_360(angle_deg: float) -> float:
    """Привести угол к диапазону 0 <= angle < 360.

    >>> normalize_360(-90)
    270.0
    >>> normalize_360(720.5)
    0.5
    """
    return float(angle_deg) % 360.0


@dataclass
class FollowConfig:
    """Настройки контура слежения."""

    counts_per_rev: int = 131072      # 17-битный энкодер
    invert_encoder: bool = True       # команда «вперёд» уменьшает счёт

    #: Круговая ось или ось с ограниченным ходом.
    #:
    #: True  — угол сводится к 0..360, ошибка по кратчайшей дуге. Годится
    #:         для оси, которая крутится свободно.
    #: False — угол разворачивается многооборотно, ошибка считается прямой
    #:         разностью. Обязательно там, где стоит редуктор: сустав на
    #:         ±90° при передаточном 10 требует ±900° на валу мотора, и
    #:         сведение по модулю оборота потеряло бы, на каком витке ось.
    circular: bool = True

    # ═══ ЗАЩИТА ОСИ ═══
    #
    # Ограничение цели — только первый слой. Он не спасает, если ось поехала
    # не в ту сторону: цель останется в пределах, а вал уйдёт в упор. Поэтому
    # проверяется и фактическое положение, и само направление движения.

    #: Жёсткие пределы хода в градусах ВАЛА от нуля. None — предела нет.
    min_motor_deg: float | None = None
    max_motor_deg: float | None = None

    #: Насколько можно выйти за предел, прежде чем это считается аварией.
    #: Ноль здесь не годится: ось всегда чуть перелетает из-за дискретности
    #: задания скорости, и авария срабатывала бы на каждом подходе к краю.
    limit_margin_deg: float = 3.0

    # ═══ МЕХАНИЧЕСКИЙ УПОР ═══
    #
    # Пределы выше отсчитываются от программного нуля и уезжают вместе с
    # ним при каждой новой калибровке середины. Для настоящего упора это
    # не годится: он привязан к механике и не должен зависеть от того,
    # где мы сегодня поставили середину.
    #
    # Поэтому здесь отдельная шкала — абсолютный угол привода 0..360°,
    # тот самый, что показывает штатная программа. Он считается прямо из
    # импульсов энкодера и от наших калибровок не зависит вовсе.

    #: Ноль абсолютной шкалы в импульсах. None — упор не задан.
    abs_zero_counts: int | None = None
    abs_invert: bool = True         # как в штатной программе
    abs_min_deg: float = 0.0        # разрешённая дуга, от
    abs_max_deg: float = 360.0      # и до

    #: Сколько проходов можно возвращаться в дугу, прежде чем признать,
    #: что ось упёрлась и дальше ползти бессмысленно.
    arc_return_passes: int = 40

    #: Сколько проходов подряд ошибка может РАСТИ при работающем приводе,
    #: прежде чем это признаётся движением не в ту сторону. Ловит
    #: перевёрнутый знак энкодера и неверный reverse — то, из-за чего ось
    #: разгоняется в упор вместо подхода к цели.
    runaway_passes: int = 6

    kp: float = 0.225                 # об/мин на градус ошибки
    min_rpm: int = 1                  # привод принимает целые об/мин
    max_rpm: int = 20
    tolerance_deg: float = 2.0        # зона удержания

    smoothing: float = 0.35           # сглаживание цели, 0..1
    deadband_deg: float = 0.5         # дрожание источника ниже этого игнорируем

    #: Упреждение по скорости цели. Чисто пропорциональный контур при слежении
    #: за движущейся целью всегда отстаёт: чтобы ехать, ему нужна ошибка. При
    #: цели 60 град/с и kp=0.225 установившееся отставание — около 44°.
    #: Упреждение задаёт скорость цели напрямую, и ошибка нужна только на
    #: подчистку остатка.
    feedforward: bool = True
    feedforward_smoothing: float = 0.4   # скорость цели шумная, её сглаживаем

    #: Насколько об/мин может измениться за один проход контура.
    #:
    #: Без этого команда скорости прыгает с нуля до потолка мгновенно.
    #: Привод отрабатывает такой скачок рывком, а рывок на повышающей
    #: передаче — прямой путь к сорванному ремню. Ограничение делает
    #: разгон и торможение плавными, ничего не отнимая у точности:
    #: у цели ось всё равно оказывается, просто без удара.
    max_rpm_step: float = 4.0

    period_s: float = 0.08            # период прохода контура

    #: Сколько проходов подряд может сорваться, прежде чем контур сдастся.
    #: Одиночный потерянный ответ на RS-485 — рядовое событие (помеха от
    #: силовой части, занятый привод), и ронять из-за него весь сеанс нельзя.
    max_consecutive_errors: int = 5

    #: Пауза после HOLD перед сменой направления. Взято из рабочей программы
    #: C30 Servo GUI: реверс подаётся не встык к противоположной команде, а
    #: через удержание, чтобы привод успел погасить скорость.
    reversal_pause_s: float = 0.04

    def __post_init__(self) -> None:
        if not 0 < self.smoothing <= 1:
            raise ValueError("smoothing в диапазоне (0, 1]")
        if self.min_rpm < 1:
            raise ValueError("привод принимает целые об/мин, минимум 1")
        if self.max_rpm < self.min_rpm:
            raise ValueError("max_rpm должен быть не меньше min_rpm")

    def travel_per_pass_deg(self, rpm: float) -> float:
        """Сколько градусов вал пройдёт за один проход контура."""
        return rpm * 6.0 * self.period_s

    def check_tolerance(self) -> str | None:
        """Предупреждение, если допуск недостижим. None — если всё в порядке."""
        floor = self.travel_per_pass_deg(self.min_rpm)
        if self.tolerance_deg < floor:
            return (f"допуск {self.tolerance_deg:.1f}° меньше пути за проход "
                    f"({floor:.2f}° на {self.min_rpm} об/мин) — ось будет "
                    f"рыскать. Возьмите не меньше {2 * floor:.1f}°.")
        return None


@dataclass
class Telemetry:
    """Снимок состояния для отображения."""

    target_deg: float = 0.0
    actual_deg: float = 0.0
    error_deg: float = 0.0
    command_rpm: int = 0
    holding: bool = True
    raw_counts: int = 0
    #: Почему ось стоит. Пусто, когда едет.
    #:
    #: Без этого «удержание» означало сразу несколько разных вещей —
    #: от «уже на месте» до «упёрлась в дугу» — и отличить их снаружи
    #: было нельзя даже при ошибке в сотню градусов.
    hold_reason: str = ""
    passes: int = 0
    period_ms: float = 0.0
    error_text: str | None = None        # фатальная: контур остановлен
    warning_text: str | None = None      # преходящая: проход сорвался
    dropped_passes: int = 0              # всего сорванных проходов за сеанс


class AngleFollower:
    """Контур слежения. Крутится в отдельном потоке, цель ставится извне."""

    def __init__(self, drive, config: FollowConfig):
        self.drive = drive
        self.config = config
        self.zero_offset_counts = 0

        self._target_deg = 0.0
        self._lock = threading.Lock()
        self._smoothed_deg: float | None = None
        self._target_rate_deg_s = 0.0
        self._commanded_rpm = 0.0         # что скомандовано сейчас, со знаком
        self._worse_passes = 0            # подряд проходов с растущей ошибкой
        self._last_actual: float | None = None
        self._out_passes = 0              # подряд проходов с растущим выходом
        self._last_overshoot: float | None = None
        self.fault: str | None = None     # причина аварийной остановки
        self.relaxed = False              # момент снят, вал крутится руками
        self.armed = False                # разрешено ли двигаться
        self.out_of_range = 0.0           # на сколько вал за пределом
        self._arc_passes = 0              # сколько возвращаемся в дугу
        self._arc_reported = False
        self._last_target_deg: float | None = None
        self._last_target_time: float | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.telemetry = Telemetry()

    # --- цель -------------------------------------------------------------

    def clamp_motor(self, degrees: float) -> float:
        """Прижать угол вала к жёстким пределам."""
        if self.config.min_motor_deg is not None:
            degrees = max(self.config.min_motor_deg, degrees)
        if self.config.max_motor_deg is not None:
            degrees = min(self.config.max_motor_deg, degrees)
        return degrees

    def beyond_limit(self, motor_deg: float) -> float:
        """На сколько градусов вал вышел за предел. 0 — в пределах."""
        margin = self.config.limit_margin_deg
        low, high = self.config.min_motor_deg, self.config.max_motor_deg
        if low is not None and motor_deg < low - margin:
            return motor_deg - (low - margin)
        if high is not None and motor_deg > high + margin:
            return motor_deg - (high + margin)
        return 0.0

    def set_target(self, degrees: float) -> None:
        """Поставить новую цель. Вызывается из потока источника.

        Цель прижимается к пределам здесь же: даже если снаружи пришло
        заведомо недопустимое значение, до привода оно не дойдёт.
        """
        value = (normalize_360(degrees) if self.config.circular
                 else self.clamp_motor(float(degrees)))
        with self._lock:
            self._target_deg = value

    def _take_target(self) -> float:
        with self._lock:
            raw = self._target_deg

        # Сглаживание по кратчайшему пути, иначе переход 359 -> 1 даёт рывок
        # через всю окружность.
        if self._smoothed_deg is None:
            self._smoothed_deg = raw
        else:
            delta = self.angle_error(raw, self._smoothed_deg)
            if abs(delta) > self.config.deadband_deg:
                moved = delta * self.config.smoothing
                self._smoothed_deg = (
                    normalize_360(self._smoothed_deg + moved)
                    if self.config.circular else self._smoothed_deg + moved)

        self._update_target_rate(self._smoothed_deg)
        return self._smoothed_deg

    def _update_target_rate(self, target_deg: float) -> None:
        """Оценить скорость движения цели, град/с."""
        now = time.monotonic()
        if self._last_target_time is not None and self._last_target_deg is not None:
            dt = now - self._last_target_time
            if dt > 0:
                moved = self.angle_error(target_deg, self._last_target_deg)
                rate = moved / dt
                alpha = self.config.feedforward_smoothing
                self._target_rate_deg_s += (rate - self._target_rate_deg_s) * alpha
        self._last_target_deg = target_deg
        self._last_target_time = now

    # --- положение --------------------------------------------------------

    def absolute_angle(self, raw: int) -> float | None:
        """Угол привода 0..360° — тот же, что показывает штатная программа.

        Считается из импульсов и своего нуля, поэтому не зависит ни от
        нашей середины, ни от того, сколько оборотов накрутили.
        """
        cfg = self.config
        if cfg.abs_zero_counts is None:
            return None
        delta = raw - cfg.abs_zero_counts
        if cfg.abs_invert:
            delta = -delta
        return (delta % cfg.counts_per_rev) * 360.0 / cfg.counts_per_rev

    def outside_arc(self, raw: int) -> str | None:
        """С какой стороны вышли за разрешённую дугу. None — внутри.

        Края считаются линейно, БЕЗ обхода через ноль. Это принципиально:
        сначала здесь искался ближайший край по окружности, и при угле
        350° ближе оказывался нижний край 0°. Ось послушно ехала вверх,
        через 360° попадала внутрь дуги, тут же выезжала снова — и так
        бесконечно, медленно вращаясь в одну сторону. У оси с упорами
        дороги через ноль нет, и делать вид, что есть, нельзя.
        """
        angle = self.absolute_angle(raw)
        if angle is None:
            return None
        cfg = self.config
        if angle > cfg.abs_max_deg:
            return "high"
        if angle < cfg.abs_min_deg:
            return "low"
        return None

    def counts_to_degrees(self, raw: int) -> float:
        """Импульсы энкодера -> градусы вала, от программного нуля.

        В круговом режиме результат лежит в 0..360. В многооборотном
        разворачивается без ограничений: три оборота назад дают -1080°.
        """
        delta = raw - self.zero_offset_counts
        if self.config.invert_encoder:
            delta = -delta
        if not self.config.circular:
            return delta * 360.0 / self.config.counts_per_rev
        return normalize_360(delta % self.config.counts_per_rev
                             * 360.0 / self.config.counts_per_rev)

    def angle_error(self, target_deg: float, actual_deg: float) -> float:
        """Ошибка с учётом режима оси."""
        if self.config.circular:
            return shortest_angle_error(target_deg, actual_deg)
        # У оси с ограниченным ходом дороги «через ноль» нет: кратчайшая
        # дуга там не короткий путь, а прыжок через всю рабочую зону.
        return target_deg - actual_deg

    def hold_here(self) -> None:
        """Сделать текущее положение целью — ось остаётся где стоит.

        Нужно везде, где движение прекращают, не выключая контур: на паузе
        и при возврате момента. Просто перестать обновлять цель мало —
        старая цель никуда не девается, и ось продолжит ехать к ней. После
        ручного перемещения это означало бы рывок обратно.
        """
        actual = self.counts_to_degrees(self.drive.read_position())
        with self._lock:
            self._target_deg = actual
        self._smoothed_deg = actual
        self._target_rate_deg_s = 0.0
        self._commanded_rpm = 0.0
        self._last_target_deg = None
        self._last_target_time = None
        self.reset_direction()

    def set_zero_here(self) -> None:
        """Принять текущее положение за 0°."""
        self.zero_offset_counts = self.drive.read_position()
        self._smoothed_deg = None
        log.info("программный ноль: raw=%d", self.zero_offset_counts)

    # --- контур -----------------------------------------------------------

    def step(self) -> None:
        """Один проход контура. Состояние направления хранится внутри.

        Нужен, когда проходами распоряжается кто-то снаружи — например
        :class:`MultiAxisFollower`, который обслуживает несколько осей на
        одной шине по очереди.
        """
        self._last_direction = self._one_pass(getattr(self, "_last_direction", None))

    def reset_direction(self) -> None:
        """Забыть направление и накопленную скорость.

        Следующий реверс пойдёт через удержание, а разгон начнётся с нуля,
        а не с той скорости, на которой ось шла до сброса.
        """
        self._last_direction = None
        self._commanded_rpm = 0.0

    def _one_pass(self, last_direction: str | None) -> str | None:
        cfg = self.config
        raw = self.drive.read_position()
        actual = self.counts_to_degrees(raw)
        target = self._take_target()
        error = self.angle_error(target, actual)

        # Пока движение не разрешено, ось только докладывает положение.
        #
        # Раньше контур начинал править положение сразу после запуска, и
        # достаточно было любого расхождения — подтолкнули руку, осел
        # ремень, сменили ноль — чтобы ось поехала, хотя пуск никто не
        # нажимал. Движение обязано требовать явного разрешения.
        if self.relaxed or not self.armed:
            self.telemetry = Telemetry(
                target_deg=target, actual_deg=actual, error_deg=error,
                command_rpm=0, holding=True, raw_counts=raw,
                passes=self.telemetry.passes + 1)
            self._last_actual = None
            self._worse_passes = 0
            self._commanded_rpm = 0.0
            if not self.relaxed:
                self.drive.hold()      # под током, но без движения
            return None

        # ── слой 2: фактическое положение вышло за предел ──────────────
        #
        # Сюда попадаем, если ось перелетела край или уехала не туда.
        # Дальше двигать нельзя ни в каком случае: гасим и поднимаем аварию.
        # Механический упор проверяется первым: он важнее программных
        # пределов, потому что за ним железо, а не настройка.
        side = self.outside_arc(raw)
        if side is not None:
            angle = self.absolute_angle(raw)
            edge = (cfg.abs_min_deg if side == "low" else cfg.abs_max_deg)
            self.out_of_range = min(abs(angle - edge), 360 - abs(angle - edge))
            # Куда ехать, чтобы вернуться в дугу.
            #
            # Направление зависит сразу от двух инверсий, и угадывать тут
            # нельзя: команда «вперёд» меняет счётчик энкодера в одну
            # сторону (invert_encoder), а абсолютная шкала считается от
            # счётчика с собственным знаком (abs_invert). Выводим явно.
            raw_per_forward = -1 if cfg.invert_encoder else 1
            angle_per_forward = (-raw_per_forward if cfg.abs_invert
                                 else raw_per_forward)
            want_increase = (side == "low")      # ушли ниже нижнего края
            reverse = (want_increase != (angle_per_forward > 0))

            # Возврат не должен длиться вечно. Если за столько проходов
            # ось не вернулась, значит она едет не туда или упёрлась в
            # железо — тогда честнее встать, чем ползти без конца.
            self._arc_passes += 1
            if self._arc_passes > cfg.arc_return_passes:
                self.drive.hold()
                self._commanded_rpm = 0.0
                self.telemetry = Telemetry(
                    target_deg=target, actual_deg=actual, error_deg=error,
                    command_rpm=0, holding=True, raw_counts=raw,
                    hold_reason=f"упор {angle:.0f}° вне "
                                f"{cfg.abs_min_deg:.0f}..{cfg.abs_max_deg:.0f}°",
                    passes=self.telemetry.passes + 1)
                if not self._arc_reported:
                    self._arc_reported = True
                    log.warning("ось стоит на упоре: %.1f° по шкале "
                                "привода, разрешено %.0f..%.0f°",
                                angle, cfg.abs_min_deg, cfg.abs_max_deg)
                return None

            self.drive.run(cfg.min_rpm, reverse=reverse)
            # Пока возвращаемся в дугу, защита от уезда молчит: движение
            # к упору она справедливо считает уездом от цели, и без этого
            # сброса контур вставал целиком.
            self._worse_passes = 0
            self._last_actual = None
            self.telemetry = Telemetry(
                target_deg=target, actual_deg=actual, error_deg=error,
                command_rpm=-cfg.min_rpm if reverse else cfg.min_rpm,
                holding=False, raw_counts=raw,
                passes=self.telemetry.passes + 1)
            return "reverse" if reverse else "forward"

        overshoot = self.beyond_limit(actual)
        if overshoot:
            # Ось за пределом. Глухо запрещать движение нельзя: тогда из
            # этого состояния не выбраться иначе как руками. Целью
            # становится сам предел, а не край допуска на перелёт: иначе
            # ось замирала бы у самой границы, в зоне нечувствительности.
            target = self.clamp_motor(actual)
            error = self.angle_error(target, actual)
            self.out_of_range = abs(overshoot)

            # Авария поднимается, только если выход РАСТЁТ: значит ось не
            # возвращается, а продолжает уходить, и это уже настоящий отказ.
            if (self._last_overshoot is not None
                    and abs(overshoot) > abs(self._last_overshoot) + 0.5):
                self._out_passes += 1
            else:
                self._out_passes = 0
            self._last_overshoot = overshoot

            if self._out_passes >= cfg.runaway_passes:
                self.drive.hold()
                self.fault = (
                    f"вал уходит за предел: {abs(overshoot):.0f}° сверх "
                    f"допустимого и продолжает расти. Положение {actual:.0f}°, "
                    f"разрешено {cfg.min_motor_deg:.0f}..{cfg.max_motor_deg:.0f}°")
                self.telemetry = Telemetry(
                    target_deg=target, actual_deg=actual, error_deg=error,
                    command_rpm=0, holding=True, raw_counts=raw,
                    passes=self.telemetry.passes + 1)
                raise DriveFault(self.fault)
        else:
            self.out_of_range = 0.0
            self._out_passes = 0
            self._last_overshoot = None
            self._arc_passes = 0
            self._arc_reported = False

        # ── слой 3: вал идёт не туда, куда скомандовано ───────────────
        #
        # Признак перевёрнутого знака именно этот: командуем «вперёд», а
        # вал уходит назад. Раньше здесь стоял рост ошибки, и это было
        # неверно: когда цель убегает быстрее, чем ось успевает — а при
        # слежении за рукой так бывает постоянно, — ошибка растёт при
        # совершенно исправной оси, и защита срабатывала впустую.
        #
        # Сравнение движения с командой от скорости цели не зависит вовсе.
        if self._last_actual is not None and abs(self._commanded_rpm) >= 1.0:
            moved = actual - self._last_actual
            if abs(moved) > 0.05:                 # мельче — шум энкодера
                if moved * self._commanded_rpm < 0:
                    self._worse_passes += 1
                else:
                    self._worse_passes = 0
            if self._worse_passes >= cfg.runaway_passes:
                self.drive.hold()
                self.fault = (
                    f"вал идёт против команды: скомандовано "
                    f"{self._commanded_rpm:+.0f} об/мин, а положение уходит "
                    f"в другую сторону, {self._worse_passes} проходов подряд. "
                    f"Перевёрнут знак — проверьте invert_encoder и reverse "
                    f"у этой оси.")
                raise DriveFault(self.fault)
        else:
            self._worse_passes = 0
        self._last_actual = actual

        # Встаём на удержание, только если цель ещё и стоит. Иначе при
        # медленном движении руки ось внутри зоны допуска отпускает привод,
        # тут же отстаёт и догоняет рывком — ход получается дёрганый.
        target_moving = (cfg.feedforward
                         and abs(self._target_rate_deg_s) >= cfg.min_rpm * 6.0)
        if abs(error) <= cfg.tolerance_deg and not target_moving:
            self.drive.hold()
            self._commanded_rpm = 0.0
            self.telemetry = Telemetry(
                target_deg=target, actual_deg=actual, error_deg=error,
                command_rpm=0, holding=True, raw_counts=raw,
                hold_reason="в допуске", passes=self.telemetry.passes + 1,
            )
            return None

        demand = error * cfg.kp
        if cfg.feedforward:
            demand += self._target_rate_deg_s / 6.0     # град/с -> об/мин

        if abs(demand) < 1e-9:
            demand = math.copysign(cfg.min_rpm, error)

        # Ограничение ускорения: скорость нарастает постепенно, а не
        # скачком. ТОРМОЖЕНИЕ И РАЗВОРОТ НЕ ОГРАНИЧИВАЕМ.
        #
        # Иначе после перелёта цели команда ещё несколько проходов
        # направлена в прежнюю сторону, пока плавно спадает через ноль.
        # Ошибка в это время растёт, ось действительно уезжает, и защита
        # справедливо принимает это за перевёрнутый знак. Сбрасывать
        # скорость надо сразу — это и безопаснее, и ремню легче, чем
        # продолжать разгон не в ту сторону.
        wanted = max(-cfg.max_rpm, min(cfg.max_rpm, demand))
        current = self._commanded_rpm
        speeding_up = (wanted * current >= 0) and abs(wanted) > abs(current)
        if speeding_up:
            step = cfg.max_rpm_step
            self._commanded_rpm = current + max(-step, min(wanted - current, step))
        else:
            self._commanded_rpm = wanted

        signed = self._commanded_rpm
        if abs(signed) < cfg.min_rpm:
            # Ниже минимума привод не умеет; тянемся к цели минимальной
            # скоростью в сторону ошибки, а не застываем.
            signed = math.copysign(cfg.min_rpm, demand if demand else 1.0)

        rpm = min(cfg.max_rpm, max(cfg.min_rpm, abs(signed)))
        rpm_int = max(cfg.min_rpm, int(round(rpm)))
        direction = "forward" if signed > 0 else "reverse"

        if last_direction is not None and direction != last_direction:
            self.drive.hold()
            time.sleep(cfg.reversal_pause_s)

        self.drive.run(rpm_int, reverse=(direction == "reverse"))

        self.telemetry = Telemetry(
            target_deg=target, actual_deg=actual, error_deg=error,
            command_rpm=rpm_int if direction == "forward" else -rpm_int,
            holding=False, raw_counts=raw,
            passes=self.telemetry.passes + 1,
        )
        return direction

    def _worker(self) -> None:
        last_direction = None
        last_start = time.monotonic()
        consecutive_errors = 0
        dropped = 0

        try:
            self.drive.enable()
        except Exception as exc:
            log.exception("привод не разрешился")
            self.telemetry.error_text = f"не удалось разрешить привод: {exc}"
            return

        while not self._stop.is_set():
            start = time.monotonic()
            try:
                last_direction = self._one_pass(last_direction)
                if consecutive_errors:
                    log.info("связь восстановилась после %d сбоев",
                             consecutive_errors)
                consecutive_errors = 0
                self.telemetry.warning_text = None
            except Exception as exc:
                # Сорванный проход — не повод останавливать сеанс. Направление
                # сбрасываем: неизвестно, дошла команда или нет, поэтому на
                # следующем проходе реверс пойдёт через HOLD.
                consecutive_errors += 1
                dropped += 1
                last_direction = None
                log.warning("проход сорвался (%d подряд): %s",
                            consecutive_errors, exc)
                self.telemetry.warning_text = (
                    f"сбой связи ({consecutive_errors} подряд): {exc}")
                self.telemetry.dropped_passes = dropped

                if consecutive_errors >= self.config.max_consecutive_errors:
                    self.telemetry.error_text = (
                        f"связь потеряна: {consecutive_errors} проходов подряд "
                        f"без ответа. {exc}")
                    break

            self.telemetry.dropped_passes = dropped
            self.telemetry.period_ms = (start - last_start) * 1000
            last_start = start

            remaining = self.config.period_s - (time.monotonic() - start)
            if remaining > 0:
                self._stop.wait(remaining)

        try:
            self.drive.hold()
        except Exception:
            log.exception("не удалось остановить привод")

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("контур уже запущен")
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def __enter__(self) -> "AngleFollower":
        self.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self.stop()


class Request:
    """Поручение, выполняемое потоком контура.

    Окно кладёт его в очередь и продолжает рисовать. Готовность и ошибку
    забирает потом: ``done`` взводится в любом случае, даже когда действие
    упало, иначе окно ждало бы вечно.
    """

    __slots__ = ("label", "action", "done", "error", "result")

    def __init__(self, label: str, action):
        self.label = label
        self.action = action
        self.done = threading.Event()
        self.error: str | None = None
        self.result = None

    def run(self) -> None:
        try:
            self.result = self.action()
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            log.exception("поручение «%s» не выполнено", self.label)
        finally:
            self.done.set()


class MultiAxisFollower:
    """Несколько осей на одной шине RS-485, один поток, обход по кругу.

    Отдельный поток на ось не годится: шина одна, и потоки всё равно встанут
    в очередь на замке, но моменты обмена станут непредсказуемыми. Здесь оси
    обслуживаются строго по очереди, поэтому период обхода известен и
    одинаков для всех.
    """

    def __init__(self, followers: dict[str, AngleFollower],
                 period_s: float = 0.12,
                 max_consecutive_errors: int = 5):
        if not followers:
            raise ValueError("нужна хотя бы одна ось")
        self.followers = followers
        self.period_s = period_s
        self.max_consecutive_errors = max_consecutive_errors

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error_text: str | None = None
        self.warning_text: str | None = None
        self._fault_axis: str | None = None
        self.sweeps = 0
        self.dropped = 0
        self.sweep_ms = 0.0
        self._requests: queue.Queue[Request] = queue.Queue()

    # --- поручения потоку контура ----------------------------------------

    def submit(self, label: str, action) -> "Request":
        """Отдать поручение потоку контура и сразу вернуть управление.

        Кнопки окна — «середина здесь», пауза, снять момент — трогают шину.
        Из потока Tk это делать нельзя: шина занята обходом, замок держится
        весь обмен, и окно на всё это время перестаёт отвечать. Три чтения
        позиции подряд при вялом переходнике — это уже секунды, и система
        считает программу зависшей.

        Поручение выполняется между обходами, когда шина свободна.
        Результат вызывающий забирает по готовности, не блокируя окно.
        """
        request = Request(label, action)
        if self._thread is not None and self._thread.is_alive():
            self._requests.put(request)
        else:
            # Контур не крутится — очередь разбирать некому. Всё равно не в
            # потоке окна: обмен по шине долгий и без обхода.
            threading.Thread(target=request.run, daemon=True).start()
        return request

    def _serve_requests(self) -> None:
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            request.run()

    def ensure_running(self) -> None:
        """Поднять поток контура, если он остановлен.

        Нужен, потому что «стоп» и авария поток гасят, а продолжить работу
        после них человек вправе, не перезапуская программу.
        """
        if self._thread is None or not self._thread.is_alive():
            self._thread = None
            self.start()

    def clear_fault(self) -> bool:
        """Снять аварию и снова запустить контур.

        Оси остаются там, где стоят: целью становится текущее положение.
        Если ось за пределом, она вернётся в зону сама — за предел
        отпускать её больше не будут.

        Возвращает False, если снимать было нечего.
        """
        if not self.error_text and not self._fault_axis:
            return False

        for follower in self.followers.values():
            follower.fault = None
            follower._worse_passes = 0
            follower._last_actual = None
            follower._out_passes = 0
            follower._last_overshoot = None

        self.error_text = None
        self.warning_text = None
        self._fault_axis = None
        self.set_armed(False)          # после аварии пуск только вручную

        if self._thread is None:
            self.freeze()
            self.start()
        else:
            self.freeze()
        return True

    def freeze(self) -> None:
        """Остановить все оси там, где они стоят."""
        for name, follower in self.followers.items():
            try:
                follower.hold_here()
                follower.drive.hold()
            except Exception as exc:
                log.exception("ось %s: не удалось остановить", name)
                self.warning_text = f"ось {name}: {exc}"

    def set_armed(self, armed: bool) -> None:
        """Разрешить или запретить движение всем осям.

        Запрет — не то же самое, что снятый момент: ось остаётся под током
        и держит позицию, просто не пытается никуда ехать.
        """
        for follower in self.followers.values():
            if not armed:
                follower._commanded_rpm = 0.0
            follower.armed = armed
        log.info("движение %s", "разрешено" if armed else "запрещено")

    @property
    def armed(self) -> bool:
        return any(f.armed for f in self.followers.values())

    def set_relaxed(self, relaxed: bool) -> None:
        """Снять или вернуть момент на всех осях.

        Возврат момента НЕ трогает цель: пока вал крутили руками, он ушёл,
        и ось рванула бы к прежней цели. Цель обновит источник, а до тех
        пор вызывающий код обязан держать паузу.
        """
        if relaxed:
            self.set_armed(False)
        for name, follower in self.followers.items():
            try:
                if relaxed:
                    follower.drive.release()
                else:
                    # Вал крутили руками: прежде чем вернуть момент, делаем
                    # целью текущее положение, иначе ось рванёт к старой.
                    follower.hold_here()
                    follower.drive.enable()
                follower.relaxed = relaxed
                follower.reset_direction()
            except Exception as exc:
                log.exception("ось %s: не удалось %s момент", name,
                              "снять" if relaxed else "вернуть")
                self.warning_text = f"ось {name}: {exc}"

    def set_targets(self, targets: dict[str, float]) -> None:
        for name, degrees in targets.items():
            self.followers[name].set_target(degrees)

    def hold_here(self) -> None:
        """Сделать текущее положение целью — ось остаётся где стоит.

        Нужно везде, где движение прекращают, не выключая контур: на паузе
        и при возврате момента. Просто перестать обновлять цель мало —
        старая цель никуда не девается, и ось продолжит ехать к ней. После
        ручного перемещения это означало бы рывок обратно.
        """
        actual = self.counts_to_degrees(self.drive.read_position())
        with self._lock:
            self._target_deg = actual
        self._smoothed_deg = actual
        self._target_rate_deg_s = 0.0
        self._commanded_rpm = 0.0
        self._last_target_deg = None
        self._last_target_time = None
        self.reset_direction()

    def set_zero_here(self) -> None:
        for follower in self.followers.values():
            follower.set_zero_here()

    def _worker(self) -> None:
        consecutive = 0
        last_sweep = time.monotonic()

        for name, follower in self.followers.items():
            try:
                follower.drive.enable()
            except Exception as exc:
                self.error_text = f"ось {name} не разрешилась: {exc}"
                return

        while not self._stop.is_set():
            start = time.monotonic()
            failed = []

            self._serve_requests()

            for name, follower in self.followers.items():
                if self._stop.is_set():
                    break
                try:
                    follower.step()
                except DriveFault as exc:
                    # Защита сработала. Связь тут ни при чём, повторять
                    # нечего: гасим всё и выходим.
                    self._fault_axis = name
                    self.set_armed(False)
                    self.error_text = f"ЗАЩИТА, ось {name}: {exc}"
                    log.error(self.error_text)
                    break
                except Exception as exc:
                    failed.append(f"{name}: {exc}")
                    follower.reset_direction()

            if self._fault_axis:
                break

            if failed:
                consecutive += 1
                self.dropped += 1
                self.warning_text = f"сбой связи ({consecutive} подряд): {'; '.join(failed)}"
                log.warning(self.warning_text)
                if consecutive >= self.max_consecutive_errors:
                    self.error_text = (f"связь потеряна: {consecutive} обходов "
                                       f"подряд со сбоями. {'; '.join(failed)}")
                    break
            else:
                consecutive = 0
                self.warning_text = None

            self.sweeps += 1
            self.sweep_ms = (start - last_sweep) * 1000
            last_sweep = start

            remaining = self.period_s - (time.monotonic() - start)
            if remaining > 0:
                self._stop.wait(remaining)

        for name, follower in self.followers.items():
            try:
                follower.drive.hold()
            except Exception:
                log.exception("ось %s не встала на удержание", name)

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("контур уже запущен")
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def __enter__(self) -> "MultiAxisFollower":
        self.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self.stop()
