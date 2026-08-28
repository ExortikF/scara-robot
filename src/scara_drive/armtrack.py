"""Углы руки по трём точкам: плечо, локоть, запястье.

Ключевое решение, от которого зависит вся надёжность: углы считаются **из
векторов между точками**, а не из положений точек в кадре.

Почему это важно. Если отображать координаты точки в угол (как делает
управление «рука влево-вправо»), то результат зависит от того, где человек
стоит, насколько близко к камере и куда её повернули. Стоит шагнуть вбок —
и ось уезжает, хотя рука не изменила позы. Векторы от этого свободны:

* ``q1`` — угол вектора плечо->локоть. Зависит только от поворота плеча.
* ``q2`` — угол между вектором локоть->запястье и вектором плечо->локоть,
  то есть насколько согнут локоть. Зависит только от сгиба.

Оба инварианта к сдвигу человека в кадре и к масштабу (расстоянию до камеры).
Это ровно та пара углов, которой описывается двухзвенная SCARA, так что
пересчёта между «человеческими» и «машинными» углами не нужно вообще.

Система координат кадра: X вправо, Y вниз. Углы приводятся к привычным
математическим — против часовой стрелки на экране положительно.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

Point = tuple[float, float]


@dataclass(frozen=True)
class ArmAngles:
    """Поза руки в плоскости кадра."""

    q1_deg: float          # плечо: абсолютный угол первого звена
    q2_deg: float          # локоть: угол второго звена относительно первого
    upper_len: float       # длина плеча в пикселях (или в долях кадра)
    fore_len: float        # длина предплечья

    @property
    def length_ratio(self) -> float:
        """Отношение предплечья к плечу. У человека почти постоянно."""
        return self.fore_len / self.upper_len if self.upper_len else 0.0


def _angle_of(dx: float, dy: float) -> float:
    """Угол вектора в градусах, против часовой стрелки на экране."""
    return math.degrees(math.atan2(-dy, dx))


def wrap180(angle_deg: float) -> float:
    """Привести угол к диапазону [-180, 180).

    >>> wrap180(190)
    -170.0
    >>> wrap180(-190)
    170.0
    >>> wrap180(45)
    45.0
    """
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


def angles_from_points(shoulder: Point, elbow: Point, wrist: Point) -> ArmAngles:
    """Посчитать углы руки по трём точкам.

    Рука вытянута вправо по экрану — оба угла нулевые:

    >>> a = angles_from_points((0, 0), (10, 0), (20, 0))
    >>> round(a.q1_deg, 6), round(a.q2_deg, 6)
    (0.0, 0.0)

    Плечо повёрнуто вверх на 90°, локоть прямой:

    >>> a = angles_from_points((0, 0), (0, -10), (0, -20))
    >>> round(a.q1_deg, 6), round(a.q2_deg, 6)
    (90.0, 0.0)

    Локоть согнут на 90° при неподвижном плече:

    >>> a = angles_from_points((0, 0), (10, 0), (10, -10))
    >>> round(a.q1_deg, 6), round(a.q2_deg, 6)
    (0.0, 90.0)

    Сдвиг человека в кадре и масштаб на углы не влияют:

    >>> b = angles_from_points((300, 250), (340, 250), (340, 210))
    >>> round(b.q1_deg, 6), round(b.q2_deg, 6)
    (0.0, 90.0)
    """
    ux, uy = elbow[0] - shoulder[0], elbow[1] - shoulder[1]
    fx, fy = wrist[0] - elbow[0], wrist[1] - elbow[1]

    upper_len = math.hypot(ux, uy)
    fore_len = math.hypot(fx, fy)
    if upper_len == 0 or fore_len == 0:
        raise ValueError("точки совпадают — угол не определён")

    q1 = _angle_of(ux, uy)
    q2 = wrap180(_angle_of(fx, fy) - q1)
    return ArmAngles(q1_deg=q1, q2_deg=q2,
                     upper_len=upper_len, fore_len=fore_len)


@dataclass
class ArmFilter:
    """Сглаживание и отсев заведомо неверных распознаваний.

    Детектор изредка выдаёт правдоподобную, но неправильную позу — например
    принимает за локоть что-то в фоне. Такой кадр отличается тем, что длина
    звена скачком меняется: рука не может за 30 миллисекунд стать вдвое
    короче.

    Сравнение идёт **с предыдущим кадром, а не со средним за всё время**. Это
    принципиально: когда рука наклоняется от плоскости кадра, её проекция
    укорачивается по-настоящему, и от долговременного среднего длина уезжает
    законно. А вот скачок между соседними кадрами физически невозможен —
    именно он и выдаёт сбой детектора.
    """

    median_window: int = 5          # кадров в медианном окне
    smoothing: float = 0.45         # 0..1, экспоненциальное сглаживание
    max_len_change: float = 0.25    # скачок длины звена за кадр
    max_jump_deg: float = 45.0      # скачок угла за кадр, больше — отбросить

    _q1: deque = field(default_factory=lambda: deque(maxlen=5), init=False)
    _q2: deque = field(default_factory=lambda: deque(maxlen=5), init=False)
    _lens: tuple[float, float] | None = field(default=None, init=False)
    _out: tuple[float, float] | None = field(default=None, init=False)

    rejected: int = field(default=0, init=False)
    accepted: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        self._q1 = deque(maxlen=self.median_window)
        self._q2 = deque(maxlen=self.median_window)

    @property
    def last(self) -> tuple[float, float] | None:
        """Последний принятый результат (q1, q2)."""
        return self._out

    def reset(self) -> None:
        self._q1.clear()
        self._q2.clear()
        self._lens = None
        self._out = None

    def update(self, angles: ArmAngles) -> tuple[float, float] | None:
        """Принять измерение. Вернуть сглаженные (q1, q2) или None, если отбросили."""
        lens = (angles.upper_len, angles.fore_len)

        if self._lens is not None:
            for was, now in zip(self._lens, lens):
                if was and abs(now - was) / was > self.max_len_change:
                    self.rejected += 1
                    return None

        if self._out is not None:
            if (abs(wrap180(angles.q1_deg - self._out[0])) > self.max_jump_deg
                    or abs(wrap180(angles.q2_deg - self._out[1])) > self.max_jump_deg):
                self.rejected += 1
                return None

        self.accepted += 1
        self._lens = lens

        self._q1.append(angles.q1_deg)
        self._q2.append(angles.q2_deg)
        median = (_circular_median(self._q1), _circular_median(self._q2))

        if self._out is None:
            self._out = median
        else:
            self._out = (
                self._out[0] + wrap180(median[0] - self._out[0]) * self.smoothing,
                self._out[1] + wrap180(median[1] - self._out[1]) * self.smoothing,
            )
        return self._out


def _circular_median(values) -> float:
    """Медиана углов с учётом того, что 179° и -179° рядом.

    >>> round(_circular_median([170.0, 175.0, -175.0]), 3)
    175.0
    >>> round(_circular_median([10.0, 12.0, 14.0]), 3)
    12.0
    """
    items = list(values)
    if not items:
        raise ValueError("пустая выборка")
    base = items[len(items) // 2]
    unwrapped = sorted(base + wrap180(v - base) for v in items)
    middle = len(unwrapped) // 2
    if len(unwrapped) % 2:
        return wrap180(unwrapped[middle])
    return wrap180((unwrapped[middle - 1] + unwrapped[middle]) / 2)
