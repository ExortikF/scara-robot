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
    error_text: str | None = None


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
        self._last_target_deg: float | None = None
        self._last_target_time: float | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.telemetry = Telemetry()

    # --- цель -------------------------------------------------------------

    def set_target(self, degrees: float) -> None:
        """Поставить новую цель. Вызывается из потока источника."""
        with self._lock:
            self._target_deg = normalize_360(degrees)

    def _take_target(self) -> float:
        with self._lock:
            raw = self._target_deg

        # Сглаживание по кратчайшему пути, иначе переход 359 -> 1 даёт рывок
        # через всю окружность.
        if self._smoothed_deg is None:
            self._smoothed_deg = raw
        else:
            delta = shortest_angle_error(raw, self._smoothed_deg)
            if abs(delta) > self.config.deadband_deg:
                self._smoothed_deg = normalize_360(
                    self._smoothed_deg + delta * self.config.smoothing
                )

        self._update_target_rate(self._smoothed_deg)
        return self._smoothed_deg

    def _update_target_rate(self, target_deg: float) -> None:
        """Оценить скорость движения цели, град/с."""
        now = time.monotonic()
        if self._last_target_time is not None and self._last_target_deg is not None:
            dt = now - self._last_target_time
            if dt > 0:
                moved = shortest_angle_error(target_deg, self._last_target_deg)
                rate = moved / dt
                alpha = self.config.feedforward_smoothing
                self._target_rate_deg_s += (rate - self._target_rate_deg_s) * alpha
        self._last_target_deg = target_deg
        self._last_target_time = now

    # --- положение --------------------------------------------------------

    def counts_to_degrees(self, raw: int) -> float:
        delta = raw - self.zero_offset_counts
        if self.config.invert_encoder:
            delta = -delta
        return normalize_360(delta % self.config.counts_per_rev
                             * 360.0 / self.config.counts_per_rev)

    def set_zero_here(self) -> None:
        """Принять текущее положение за 0°."""
        self.zero_offset_counts = self.drive.read_position()
        self._smoothed_deg = None
        log.info("программный ноль: raw=%d", self.zero_offset_counts)

    # --- контур -----------------------------------------------------------

    def _one_pass(self, last_direction: str | None) -> str | None:
        cfg = self.config
        raw = self.drive.read_position()
        actual = self.counts_to_degrees(raw)
        target = self._take_target()
        error = shortest_angle_error(target, actual)

        if abs(error) <= cfg.tolerance_deg:
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
        try:
            self.drive.enable()
            while not self._stop.is_set():
                start = time.monotonic()
                last_direction = self._one_pass(last_direction)
                self.telemetry.period_ms = (start - last_start) * 1000
                last_start = start

                remaining = self.config.period_s - (time.monotonic() - start)
                if remaining > 0:
                    self._stop.wait(remaining)
        except Exception as exc:
            log.exception("контур слежения остановлен")
            self.telemetry.error_text = str(exc)
        finally:
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
