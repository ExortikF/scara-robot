"""Повтор движений руки: сглаживание, ограничение и выдача в привод.

Задача принципиально другая, чем траекторное движение по программе. Здесь нет
заранее известного пути — есть поток целей, приходящий с трекера примерно 30 раз
в секунду, шумный и рваный. Робот обязан за ним идти, но не обязан повторять
дрожание руки и не имеет права разгоняться быстрее своих пределов.

Между трекером и приводом стоит формирователь :class:`MotionShaper`. Он:

1. давит дрожание (зона нечувствительности + экспоненциальное сглаживание);
2. держит угол в пределах сустава;
3. ограничивает скорость и ускорение.

Пункт 3 — не украшение. Оператор дёргает рукой мгновенно, механика так не может;
без ограничителя вы получите срыв, потерю импульсов и разъезд координат.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Protocol

from .pulses import JointPlan

log = logging.getLogger(__name__)


@dataclass
class JointLimits:
    """Пределы одного сустава."""

    min_deg: float
    max_deg: float
    max_deg_s: float = 120.0        # скорость
    max_deg_s2: float = 400.0       # ускорение
    deadband_deg: float = 0.15      # ниже этого цель считается неизменной
    smoothing: float = 0.35         # 0..1: меньше — плавнее и медленнее

    def __post_init__(self) -> None:
        if self.min_deg >= self.max_deg:
            raise ValueError("min_deg должен быть меньше max_deg")
        if not 0 < self.smoothing <= 1:
            raise ValueError("smoothing в диапазоне (0, 1]")


class MotionShaper:
    """Формирователь для одного сустава. Держит состояние между кадрами."""

    def __init__(self, limits: JointLimits, start_deg: float = 0.0):
        self.limits = limits
        self.target_deg = start_deg      # сглаженная цель
        self.command_deg = start_deg     # то, что реально отдано в привод
        self.velocity_deg_s = 0.0

    def update(self, raw_target_deg: float, dt: float) -> float:
        """Принять сырую цель с трекера, вернуть новый командный угол."""
        if dt <= 0:
            return self.command_deg
        lim = self.limits

        raw = max(lim.min_deg, min(raw_target_deg, lim.max_deg))
        if abs(raw - self.target_deg) > lim.deadband_deg:
            self.target_deg += (raw - self.target_deg) * lim.smoothing

        error = self.target_deg - self.command_deg

        # Скорость, с которой ещё успеваем затормозить до цели:
        # v = sqrt(2*a*s). Без этого команда проскакивает и начинает колебаться.
        stopping = math.sqrt(2 * lim.max_deg_s2 * abs(error))
        desired = math.copysign(min(lim.max_deg_s, stopping), error)

        # Ограничение ускорения.
        max_change = lim.max_deg_s2 * dt
        delta_v = max(-max_change, min(desired - self.velocity_deg_s, max_change))
        self.velocity_deg_s += delta_v

        self.command_deg += self.velocity_deg_s * dt
        self.command_deg = max(lim.min_deg, min(self.command_deg, lim.max_deg))
        return self.command_deg


class PulseSink(Protocol):
    """Куда уходят импульсы. Реализация зависит от железа."""

    def send(self, axis: int, steps: int, step_hz: int) -> None: ...


class DryRunSink:
    """Заглушка: копит команды вместо отправки. Для проверки без железа."""

    def __init__(self) -> None:
        self.commands: list[tuple[int, int, int]] = []

    def send(self, axis: int, steps: int, step_hz: int) -> None:
        self.commands.append((axis, steps, step_hz))

    @property
    def net_steps(self) -> dict[int, int]:
        result: dict[int, int] = {}
        for axis, steps, _ in self.commands:
            result[axis] = result.get(axis, 0) + steps
        return result


@dataclass
class TeleopAxis:
    """Сустав в контуре телеуправления."""

    axis: int
    plan: JointPlan
    limits: JointLimits
    shaper: MotionShaper = field(init=False)
    _sent_steps: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        self.shaper = MotionShaper(self.limits)

    def feasible_max_deg_s(self, max_step_hz: int) -> float:
        """Скорость сустава, которую физически вытягивает импульсный тракт."""
        return max_step_hz / float(self.plan.pulses_per_degree)

    def fit_to(self, max_step_hz: int) -> None:
        """Урезать предел скорости, если он выше возможностей тракта.

        Иначе формирователь разгонит сустав до недостижимой скорости, кадры
        начнут не укладываться в свой интервал, и робот поедет с нарастающим
        отставанием от руки, хотя импульсы дойдут все до одного.
        """
        feasible = self.feasible_max_deg_s(max_step_hz)
        if self.limits.max_deg_s > feasible:
            log.warning(
                "ось %d: предел %.0f град/с недостижим при %d Гц и %.0f имп./град "
                "— снижен до %.0f град/с",
                self.axis, self.limits.max_deg_s, max_step_hz,
                float(self.plan.pulses_per_degree), feasible,
            )
            self.limits.max_deg_s = feasible

    def step_delta(self, raw_target_deg: float, dt: float) -> tuple[int, int]:
        """Вернуть (импульсы, частота) для этого кадра.

        Импульсы считаются от накопленного отправленного количества, а не от
        угла кадра. Дробный остаток не теряется, поэтому ошибка округления не
        накапливается — при 30 кадрах в секунду это принципиально.
        """
        command_deg = self.shaper.update(raw_target_deg, dt)
        want_steps = round(command_deg * float(self.plan.pulses_per_degree))
        delta = want_steps - self._sent_steps
        if delta == 0:
            return 0, 0
        self._sent_steps = want_steps
        step_hz = max(1, round(abs(delta) / dt))
        return delta, step_hz


class TeleopSession:
    """Контур «трекер -> суставы -> привод».

    >>> from .pulses import JointPlan
    >>> plan = JointPlan(name="J1", reduction=50, pulses_per_rev=3600)
    >>> sink = DryRunSink()
    >>> axis = TeleopAxis(axis=0, plan=plan, limits=JointLimits(-90, 90))
    >>> session = TeleopSession([axis], sink, max_step_hz=20_000)
    >>> for _ in range(200):                      # цель 30 градусов, 30 кадров/с
    ...     session.update({0: 30.0}, dt=1 / 30)

    Приходим к цели с точностью до зоны нечувствительности — она для того и
    нужна, остаточное смещение здесь не ошибка:

    >>> abs(axis.shaper.command_deg - 30) <= axis.limits.deadband_deg
    True
    >>> sink.net_steps[0] == round(axis.shaper.command_deg
    ...                            * float(plan.pulses_per_degree))
    True

    Ни один кадр не пришлось обрезать по частоте:

    >>> session.clipped_frames
    0
    """

    #: Во сколько раз частота должна превысить потолок, чтобы это стоило
    #: сообщения. Округление размера кадра даёт превышение на доли процента.
    CLIP_WARN_RATIO = 1.05

    def __init__(self, axes: list[TeleopAxis], sink: PulseSink,
                 max_step_hz: int = 20_000):
        self.axes = {a.axis: a for a in axes}
        self.sink = sink
        self.max_step_hz = max_step_hz
        for axis in axes:
            axis.fit_to(max_step_hz)
        self._last_time: float | None = None
        self.clipped_frames = 0

    def update(self, targets: dict[int, float], dt: float | None = None) -> None:
        """Один кадр: цели по суставам в градусах."""
        now = time.monotonic()
        if dt is None:
            dt = now - self._last_time if self._last_time else 1 / 30
        self._last_time = now

        for index, raw_deg in targets.items():
            axis = self.axes.get(index)
            if axis is None:
                raise KeyError(f"ось {index} не описана в сеансе")
            steps, step_hz = axis.step_delta(raw_deg, dt)
            if steps == 0:
                continue
            if step_hz > self.max_step_hz:
                # Импульсы при этом не теряются: МК выдаст все, просто отрезок
                # займёт чуть больше кадра. Опасно не само обрезание, а его
                # систематичность — тогда отставание от руки копится.
                if step_hz > self.max_step_hz * self.CLIP_WARN_RATIO:
                    self.clipped_frames += 1
                    log.warning("ось %d: %d Гц выше потолка %d — кадр растянут",
                                index, step_hz, self.max_step_hz)
                step_hz = self.max_step_hz
            self.sink.send(index, steps, step_hz)
