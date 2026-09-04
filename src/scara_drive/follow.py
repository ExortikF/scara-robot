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
        self._worse_passes = 0            # подряд проходов с растущей ошибкой
        self._last_abs_error: float | None = None
        self.fault: str | None = None     # причина аварийной остановки
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
        """Забыть текущее направление: следующий реверс пойдёт через HOLD."""
        self._last_direction = None

    def _one_pass(self, last_direction: str | None) -> str | None:
        cfg = self.config
        raw = self.drive.read_position()
        actual = self.counts_to_degrees(raw)
        target = self._take_target()
        error = self.angle_error(target, actual)

        # ── слой 2: фактическое положение вышло за предел ──────────────
        #
        # Сюда попадаем, если ось перелетела край или уехала не туда.
        # Дальше двигать нельзя ни в каком случае: гасим и поднимаем аварию.
        overshoot = self.beyond_limit(actual)
        if overshoot:
            self.drive.hold()
            self.fault = (
                f"вал вышел за предел на {abs(overshoot):.1f}° "
                f"(положение {actual:.0f}°, допустимо "
                f"{cfg.min_motor_deg:.0f}..{cfg.max_motor_deg:.0f}°)")
            self.telemetry = Telemetry(
                target_deg=target, actual_deg=actual, error_deg=error,
                command_rpm=0, holding=True, raw_counts=raw,
                passes=self.telemetry.passes + 1)
            raise DriveFault(self.fault)

        # ── слой 3: ось едет не в ту сторону ──────────────────────────
        #
        # Привод работает, а ошибка растёт проход за проходом — значит знак
        # где-то перевёрнут. Это самый опасный отказ: ось разгоняется в
        # упор, и с редуктором момент туда приходит умноженный.
        if last_direction is not None:
            magnitude = abs(error)
            if (self._last_abs_error is not None
                    and magnitude > self._last_abs_error + 0.5):
                self._worse_passes += 1
            else:
                self._worse_passes = 0
            if self._worse_passes >= cfg.runaway_passes:
                self.drive.hold()
                self.fault = (
                    f"ось уезжает от цели: ошибка росла {self._worse_passes} "
                    f"проходов подряд и дошла до {magnitude:.0f}°. "
                    f"Похоже, перевёрнут знак — проверьте invert_encoder "
                    f"и reverse у этой оси.")
                raise DriveFault(self.fault)
            self._last_abs_error = magnitude
        else:
            self._last_abs_error = abs(error)
            self._worse_passes = 0

        # Встаём на удержание, только если цель ещё и стоит. Иначе при
        # медленном движении руки ось внутри зоны допуска отпускает привод,
        # тут же отстаёт и догоняет рывком — ход получается дёрганый.
        target_moving = (cfg.feedforward
                         and abs(self._target_rate_deg_s) >= cfg.min_rpm * 6.0)
        if abs(error) <= cfg.tolerance_deg and not target_moving:
            self.drive.hold()
            self.telemetry = Telemetry(
                target_deg=target, actual_deg=actual, error_deg=error,
                command_rpm=0, holding=True, raw_counts=raw,
                passes=self.telemetry.passes + 1,
            )
            return None

        demand = error * cfg.kp
        if cfg.feedforward:
            demand += self._target_rate_deg_s / 6.0     # град/с -> об/мин

        if abs(demand) < 1e-9:
            demand = math.copysign(cfg.min_rpm, error)

        rpm = min(cfg.max_rpm, max(cfg.min_rpm, abs(demand)))
        rpm_int = max(cfg.min_rpm, int(round(rpm)))
        direction = "forward" if demand > 0 else "reverse"

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

    def set_targets(self, targets: dict[str, float]) -> None:
        for name, degrees in targets.items():
            self.followers[name].set_target(degrees)

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

            for name, follower in self.followers.items():
                if self._stop.is_set():
                    break
                try:
                    follower.step()
                except DriveFault as exc:
                    # Защита сработала. Связь тут ни при чём, повторять
                    # нечего: гасим всё и выходим.
                    self._fault_axis = name
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
