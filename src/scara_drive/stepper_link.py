"""Сторона ноутбука: планирование разгона и отправка отрезков в МК.

Ноутбук не генерирует импульсы — он режет перемещение на отрезки постоянной
частоты и отдаёт их микроконтроллеру, который отсчитывает импульсы железным
таймером. Почему именно так — в ``docs/realtime_pulses.md``.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass

log = logging.getLogger(__name__)

MIN_STEP_HZ = 400        # нижняя граница PIO на Pico, см. прошивку
MAX_STEP_HZ = 200_000


class LinkError(RuntimeError):
    """МК не ответил или ответил отказом."""


@dataclass(frozen=True)
class Segment:
    """Отрезок движения: сколько импульсов и с какой частотой."""

    steps: int          # знак задаёт направление
    step_hz: int

    @property
    def duration_s(self) -> float:
        return abs(self.steps) / self.step_hz


@dataclass
class RampProfile:
    """Трапеция скорости."""

    max_step_hz: int = 20_000
    start_step_hz: int = MIN_STEP_HZ
    accel_steps_s2: float = 200_000.0   # имп./с²
    ramp_segments: int = 12             # на сколько ступенек резать разгон

    def __post_init__(self) -> None:
        if not MIN_STEP_HZ <= self.start_step_hz <= self.max_step_hz <= MAX_STEP_HZ:
            raise ValueError(
                f"нужно {MIN_STEP_HZ} <= start_step_hz <= max_step_hz <= {MAX_STEP_HZ}"
            )
        if self.accel_steps_s2 <= 0 or self.ramp_segments < 1:
            raise ValueError("ускорение и число ступенек должны быть положительными")


def plan_ramp(steps: int, profile: RampProfile) -> list[Segment]:
    """Разложить перемещение на ``steps`` импульсов в трапецию скорости.

    Сумма импульсов по отрезкам всегда точно равна ``steps`` — остаток от
    округления уходит в участок постоянной скорости, чтобы ось не
    недоезжала. Именно эта ошибка накапливается от команды к команде.

    >>> segs = plan_ramp(50_000, RampProfile())
    >>> sum(s.steps for s in segs)
    50000
    >>> segs[0].step_hz < segs[len(segs)//2].step_hz
    True
    """
    if steps == 0:
        return []

    sign = 1 if steps > 0 else -1
    total = abs(steps)
    v0, v1, a = profile.start_step_hz, profile.max_step_hz, profile.accel_steps_s2

    if total <= 2 * profile.ramp_segments:
        # Слишком коротко, чтобы резать: ступеньки вышли бы по паре импульсов,
        # и накладные расходы на обмен превысили бы само движение.
        return [Segment(steps, v0)]

    # Импульсов на разгон от v0 до v1: s = (v1^2 - v0^2) / 2a
    ramp_steps = (v1 * v1 - v0 * v0) / (2 * a)

    if 2 * ramp_steps > total:
        # Треугольник: до максимума не успеваем, разгоняемся до пика и тормозим.
        v1 = math.sqrt(v0 * v0 + a * total)
        v1 = max(profile.start_step_hz, min(int(v1), profile.max_step_hz))
        ramp_steps = (v1 * v1 - v0 * v0) / (2 * a)

    def stage(from_hz: float, to_hz: float) -> list[Segment]:
        """Ступеньки частоты между from_hz и to_hz."""
        out: list[Segment] = []
        n = profile.ramp_segments
        edges = [from_hz + (to_hz - from_hz) * i / n for i in range(n + 1)]
        for lo, hi in zip(edges, edges[1:]):
            count = round((hi * hi - lo * lo) / (2 * a))
            if count > 0:
                out.append(Segment(sign * count, max(MIN_STEP_HZ, round((lo + hi) / 2))))
        return out

    accel = stage(v0, v1)
    decel = [Segment(s.steps, s.step_hz) for s in reversed(stage(v0, v1))]

    used = sum(abs(s.steps) for s in accel) + sum(abs(s.steps) for s in decel)
    cruise = total - used
    if cruise < 0:
        # Округление съело больше, чем есть: срезаем торможение с конца.
        return _trim(accel + decel, total, sign)

    segments = accel
    if cruise > 0:
        segments = segments + [Segment(sign * cruise, v1)]
    segments = segments + decel

    assert sum(abs(s.steps) for s in segments) == total
    return segments


def _trim(segments: list[Segment], total: int, sign: int) -> list[Segment]:
    """Урезать хвост так, чтобы сумма импульсов была ровно total."""
    out: list[Segment] = []
    left = total
    for seg in segments:
        if left <= 0:
            break
        take = min(abs(seg.steps), left)
        out.append(Segment(sign * take, seg.step_hz))
        left -= take
    return out


def describe(segments: list[Segment]) -> str:
    """Человекочитаемый план — для ``--dry-run``."""
    if not segments:
        return "нечего делать"
    total = sum(abs(s.steps) for s in segments)
    duration = sum(s.duration_s for s in segments)
    peak = max(s.step_hz for s in segments)
    lines = [f"{'отрезок':>8} {'импульсы':>10} {'частота, Гц':>13} {'время, мс':>11}"]
    lines.append("-" * len(lines[0]))
    for i, s in enumerate(segments, 1):
        lines.append(f"{i:>8} {s.steps:>10} {s.step_hz:>13} {s.duration_s*1000:>11.1f}")
    lines.append("-" * len(lines[0]))
    lines.append(f"итого {total} имп., пик {peak} Гц, {duration:.3f} с")
    return "\n".join(lines)


class StepperLink:
    """Клиент прошивки ``pico_step`` поверх USB-serial.

    >>> link = StepperLink(port="/dev/tty.usbmodem1101")   # doctest: +SKIP
    >>> with link:                                         # doctest: +SKIP
    ...     link.move(axis=0, steps=50_000, profile=RampProfile())
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        import serial  # pyserial; импорт здесь, чтобы --dry-run работал без него

        self._serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def __enter__(self) -> "StepperLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            self.stop()
        finally:
            self._serial.close()

    def _command(self, text: str) -> str:
        log.debug(">> %s", text)
        self._serial.reset_input_buffer()
        self._serial.write((text + "\n").encode())
        reply = self._serial.readline().decode().strip()
        log.debug("<< %s", reply)
        if not reply:
            raise LinkError(f"нет ответа на {text!r}")
        if reply.startswith("err"):
            raise LinkError(f"{text!r}: {reply}")
        return reply

    def version(self) -> str:
        return self._command("?")

    def positions(self) -> dict[int, int]:
        reply = self._command("P").removeprefix("ok").strip()
        result = {}
        for item in reply.split():
            index, _, value = item.partition(":")
            result[int(index)] = int(value.rstrip("*"))
        return result

    def stop(self) -> None:
        self._command("S")

    def move(self, axis: int, steps: int, profile: RampProfile) -> None:
        """Выполнить перемещение целиком, дожидаясь каждого отрезка.

        Отрезки уходят по одному: следующий отправляется, когда предыдущий
        отработан. Пауз между ними это не исключает — на стыке ось идёт с
        постоянной частотой, поэтому дрожание USB портит время, но не
        число импульсов. Положение не теряется.
        """
        segments = plan_ramp(steps, profile)
        log.info("ось %d: %d имп. в %d отрезках", axis, steps, len(segments))
        for i, segment in enumerate(segments, 1):
            self._command(f"M {axis} {segment.steps} {segment.step_hz}")
            time.sleep(segment.duration_s)
            log.debug("отрезок %d/%d готов", i, len(segments))
