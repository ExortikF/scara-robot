"""Кинематика плоской двухзвенной SCARA (суставы J1, J2).

Плоскость XY; Z и поворот схвата к этой задаче не относятся — они идут
отдельными осями и пересчёта не требуют.

    q1 — угол первого звена от оси X
    q2 — угол второго звена относительно первого («локоть»)
"""

from __future__ import annotations

import math
from dataclasses import dataclass


class UnreachableError(ValueError):
    """Точка вне рабочей зоны."""


@dataclass(frozen=True)
class ScaraGeometry:
    """Длины звеньев, мм."""

    l1: float
    l2: float

    @property
    def reach_max(self) -> float:
        return self.l1 + self.l2

    @property
    def reach_min(self) -> float:
        return abs(self.l1 - self.l2)

    def forward(self, q1_deg: float, q2_deg: float) -> tuple[float, float]:
        """Углы суставов -> положение схвата.

        >>> g = ScaraGeometry(200, 150)
        >>> x, y = g.forward(0, 0)
        >>> round(x, 6), round(y, 6)
        (350.0, 0.0)
        """
        q1 = math.radians(q1_deg)
        q2 = math.radians(q2_deg)
        x = self.l1 * math.cos(q1) + self.l2 * math.cos(q1 + q2)
        y = self.l1 * math.sin(q1) + self.l2 * math.sin(q1 + q2)
        return x, y

    def inverse(self, x: float, y: float, *, elbow_up: bool = True
                ) -> tuple[float, float]:
        """Положение схвата -> углы суставов, градусы.

        У двухзвенника два решения — «локоть вверх» и «локоть вниз».
        Переключаться между ними на ходу нельзя: рука дёрнется через всю
        рабочую зону. Держите один флаг на весь сеанс.

        >>> g = ScaraGeometry(200, 150)
        >>> q1, q2 = g.inverse(*g.forward(30, 45))
        >>> round(q1, 6), round(q2, 6)
        (30.0, 45.0)
        """
        distance = math.hypot(x, y)
        if distance > self.reach_max or distance < self.reach_min:
            raise UnreachableError(
                f"точка ({x:.1f}, {y:.1f}) на радиусе {distance:.1f} мм вне "
                f"кольца {self.reach_min:.1f}..{self.reach_max:.1f} мм"
            )

        cos_q2 = (distance**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_q2 = max(-1.0, min(1.0, cos_q2))   # страховка от ошибок округления
        q2 = math.acos(cos_q2)
        if not elbow_up:
            q2 = -q2

        q1 = math.atan2(y, x) - math.atan2(
            self.l2 * math.sin(q2), self.l1 + self.l2 * math.cos(q2)
        )
        return math.degrees(q1), math.degrees(q2)

    def clamp_to_workspace(self, x: float, y: float, *, margin: float = 1.0
                           ) -> tuple[float, float]:
        """Притянуть точку в рабочее кольцо вместо отказа.

        Для телеуправления это важнее исключения: рука оператора неизбежно
        выходит за зону, и робот должен упереться в границу, а не встать.
        """
        distance = math.hypot(x, y)
        if distance == 0:
            return self.reach_min + margin, 0.0
        low = self.reach_min + margin
        high = self.reach_max - margin
        clamped = max(low, min(distance, high))
        scale = clamped / distance
        return x * scale, y * scale
