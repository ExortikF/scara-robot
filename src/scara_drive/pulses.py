"""Расчёт импульсного тракта: электронный редуктор, разрешение, потолок скорости.

Цепочка задания (руководство C30, стр. 46-47):

    PLC (PTO) --f1--> [эл. редуктор N/M] --f2--> внутреннее задание положения

где ``f2 = f1 * N/M``, ``N = P-028*10000 + P-029``, ``M = P-030``, а один оборот
вала мотора соответствует ``counts_per_rev`` внутренних единиц (разрешение
энкодера: 17 бит = 131072, 23 бита = 8388608).

Отсюда число импульсов от ПЛК на оборот вала:

    pulses_per_rev = counts_per_rev * M / N
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import gcd

# Разрешение энкодера, имп./об (гл. 7.1: адаптируются 17- и 23-битные)
ENCODER_17BIT = 1 << 17  # 131072
ENCODER_23BIT = 1 << 23  # 8388608

# Пределы полей эл. редуктора (гл. 4.3.1)
P028_MAX = 32000
P029_MAX = 9999
P030_MAX = 32767
GEAR_NUM_SCALE = 10000

# Потолок частоты на входе привода (гл. 2.2.2, интерфейс C3)
DRIVE_MAX_HZ_DIFFERENTIAL = 500_000
DRIVE_MAX_HZ_SINGLE_ENDED = 200_000

# Потолок частоты на выходе S7-1200 (PTO)
PLC_MAX_HZ_ONBOARD = 100_000  # быстрые встроенные DQ
PLC_MAX_HZ_SB1222 = 200_000   # сигнальная плата


@dataclass(frozen=True)
class Gear:
    """Настройка электронного редуктора привода."""

    p028: int
    p029: int
    p030: int

    @property
    def numerator(self) -> int:
        return self.p028 * GEAR_NUM_SCALE + self.p029

    @property
    def denominator(self) -> int:
        return self.p030

    @property
    def ratio(self) -> Fraction:
        return Fraction(self.numerator, self.denominator)

    def __str__(self) -> str:
        return (f"P-028={self.p028} P-029={self.p029} P-030={self.p030} "
                f"(N/M = {self.numerator}/{self.denominator})")


def gear_for_pulses_per_rev(pulses_per_rev: int,
                            counts_per_rev: int = ENCODER_17BIT) -> Gear:
    """Подобрать эл. редуктор под желаемое число импульсов ПЛК на оборот вала.

    Требуется точное представление: ``counts_per_rev / pulses_per_rev`` должно
    сокращаться в дробь, влезающую в поля P-028/P-029/P-030. Приближение не
    допускается — оно даёт накапливающуюся ошибку положения.

    >>> gear_for_pulses_per_rev(3600).ratio
    Fraction(8192, 225)
    >>> gear_for_pulses_per_rev(10000).ratio
    Fraction(8192, 625)
    """
    if pulses_per_rev <= 0:
        raise ValueError("pulses_per_rev должно быть положительным")

    divisor = gcd(counts_per_rev, pulses_per_rev)
    num = counts_per_rev // divisor
    den = pulses_per_rev // divisor

    if den > P030_MAX:
        raise ValueError(
            f"{pulses_per_rev} имп./об не выражается точно: знаменатель "
            f"{den} > P-030_max={P030_MAX}. Возьмите значение, у которого с "
            f"{counts_per_rev} больше общих множителей (обычно кратное 2)."
        )
    p028, p029 = divmod(num, GEAR_NUM_SCALE)
    if p028 > P028_MAX:
        raise ValueError(f"числитель {num} не влезает в P-028/P-029")
    return Gear(p028=p028, p029=p029, p030=den)


def pulses_per_rev(gear: Gear, counts_per_rev: int = ENCODER_17BIT) -> Fraction:
    """Сколько импульсов ПЛК даёт один оборот вала при данном редукторе."""
    return Fraction(counts_per_rev) / gear.ratio


@dataclass(frozen=True)
class JointPlan:
    """Импульсный тракт одного сустава SCARA."""

    name: str
    reduction: float          # передаточное число редуктора сустава, i:1
    pulses_per_rev: int       # импульсов ПЛК на оборот вала мотора
    counts_per_rev: int = ENCODER_17BIT
    plc_max_hz: int = PLC_MAX_HZ_ONBOARD
    motor_rated_rpm: float = 3000.0

    @property
    def gear(self) -> Gear:
        return gear_for_pulses_per_rev(self.pulses_per_rev, self.counts_per_rev)

    @property
    def pulses_per_degree(self) -> Fraction:
        """Импульсов ПЛК на градус сустава."""
        return Fraction(self.pulses_per_rev) * Fraction(self.reduction) / 360

    @property
    def degrees_per_pulse(self) -> float:
        """Разрешение сустава, град/имп."""
        return float(1 / self.pulses_per_degree)

    @property
    def integer_pulses_per_degree(self) -> bool:
        """True, если на градус приходится целое число импульсов.

        Дробное значение само по себе не ошибка — ПЛК считает в градусах и
        округляет один раз на команду, — но целое исключает разнобой между
        «доехал» по ПЛК и по приводу.
        """
        return self.pulses_per_degree.denominator == 1

    @property
    def max_motor_rpm(self) -> float:
        """Обороты вала, достижимые на потолке частоты ПЛК."""
        return self.plc_max_hz * 60 / self.pulses_per_rev

    @property
    def limited_by(self) -> str:
        return "ПЛК" if self.max_motor_rpm < self.motor_rated_rpm else "мотор"

    @property
    def max_joint_deg_s(self) -> float:
        """Максимальная скорость сустава, град/с."""
        rpm = min(self.max_motor_rpm, self.motor_rated_rpm)
        return rpm / self.reduction * 360 / 60

    def pulses_for(self, degrees: float) -> int:
        """Импульсов на перемещение сустава на ``degrees`` градусов."""
        return round(degrees * float(self.pulses_per_degree))

    def report(self) -> str:
        return "\n".join([
            f"сустав {self.name}: редуктор 1:{self.reduction:g}, "
            f"{self.pulses_per_rev} имп./об вала",
            f"  {self.gear}",
            f"  разрешение      {self.degrees_per_pulse:.5f} град/имп "
            f"({float(self.pulses_per_degree):.1f} имп./град"
            f"{'' if self.integer_pulses_per_degree else ', дробное'})",
            f"  потолок вала    {self.max_motor_rpm:.0f} об/мин "
            f"(ограничивает {self.limited_by})",
            f"  скорость        {self.max_joint_deg_s:.0f} град/с",
            f"  оборот сустава  {self.pulses_for(360)} имп.",
        ])
