"""Привод на бумаге — для проверки контура без железа.

Реализует ровно те четыре метода, которые нужны :mod:`scara_drive.follow`:
``read_position``, ``enable``, ``hold``, ``run`` (плюс ``stop`` и
``release``). Больше контуру от привода ничего не требуется, поэтому
подстановка получается полной: те же расчёты, та же логика защиты, те же
переходы — только вместо вала счётчик.

Именно на нём проверялось всё, что в проекте измерено числами:
отставание слежения, срабатывание защиты, плавность разгона.
"""

from __future__ import annotations

import time


class SimulatedAxis:
    """Ось, которая просто интегрирует заданную скорость.

    ``latency_s`` изображает время обмена по RS-485. Без него контур в
    проверках крутился бы на скорости памяти и показывал недостижимые
    на железе результаты.
    """

    def __init__(self, counts_per_rev: int = 131072, invert: bool = True,
                 latency_s: float = 0.006):
        self.counts_per_rev = counts_per_rev
        self.invert = invert
        self.latency_s = latency_s
        self.position = 0
        self.rpm = 0.0
        self._last = time.monotonic()

    def _advance(self) -> None:
        now = time.monotonic()
        dt, self._last = now - self._last, now
        counts = self.rpm / 60.0 * self.counts_per_rev * dt
        # invert=True: команда «вперёд» уменьшает показание энкодера,
        # как у привода C30.
        self.position += int(-counts if self.invert else counts)

    def read_position(self) -> int:
        time.sleep(self.latency_s)
        self._advance()
        return self.position

    def enable(self) -> None:
        pass

    def hold(self) -> None:
        self._advance()
        self.rpm = 0.0

    def stop(self) -> None:
        self.hold()

    def release(self) -> None:
        self.hold()

    def run(self, speed: int, *, reverse: bool = False) -> None:
        time.sleep(self.latency_s)
        self._advance()
        self.rpm = -speed if reverse else speed
