"""Выход в заданный угол «любой ценой»: подъехать, остановиться, проверить.

Почему не PID
-------------

Классический PID поверх Modbus упирается в три вещи, которых никаким подбором
коэффициентов не убрать:

1. **Задание скорости целое, в об/мин.** Медленнее 1 об/мин (6 град/с) привод
   ехать не умеет. У цели это означает, что за один цикл обмена (~60 мс) вал
   проскакивает ~0.4° — сравнимо с желаемой точностью.
2. **Мёртвое время.** Между «прочитал положение» и «привод отреагировал»
   проходят десятки миллисекунд. Для регулятора это чистая задержка, которая
   съедает запас устойчивости и раскачивает контур у самой цели.
3. **Момент трогания.** Тронуться с места нужнее больше, чем ехать. Регулятор
   у цели выдаёт малый выход, вал стоит, ошибка не убывает.

Отсюда и типичная картина: подъехал, проскочил, вернулся, проскочил, и так
пока не надоест.

Как работает этот модуль
------------------------

Вместо непрерывного регулирования — приближение уступами и обязательная
проверка после остановки:

* далеко  — едем на максимальной скорости;
* ближе   — скорость пропорционально ошибке;
* у цели  — короткими толчками: тронулись, проехали чуть-чуть, встали;
* внутри допуска — стоп, ждём успокоения, **перечитываем положение**.

Ключевое здесь — последний пункт. Позиция засчитывается только после
остановки, а не в момент пролёта через неё. Если после успокоения вал вне
допуска, делается ещё один заход. Настраивать нечего: алгоритм не пытается
угадать динамику, он её измеряет.

Цена — время. Выход в точку занимает секунды, а не миллисекунды. Для наладки,
раскладки по позициям и телеуправления с задержкой это нормально; для
траекторного движения нужен импульсный тракт (см. ``docs/pulse_positioning.md``).
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

log = logging.getLogger(__name__)


def normalize_360(angle_deg: float) -> float:
    """Привести угол к диапазону 0 <= a < 360.

    >>> normalize_360(370.0), normalize_360(-10.0), normalize_360(360.0)
    (10.0, 350.0, 0.0)
    """
    return float(angle_deg) % 360.0


def shortest_error(target_deg: float, actual_deg: float) -> float:
    """Ошибка по кратчайшему пути через ноль, в диапазоне [-180, 180).

    >>> shortest_error(1, 359)
    2.0
    >>> shortest_error(359, 1)
    -2.0


    Ровно противоположная точка попадает на край диапазона и всегда даёт один
    и тот же знак — направление в этом случае выбирается детерминированно, а
    не случайно:

    >>> shortest_error(180, 0)
    -180.0
    """
    return ((float(target_deg) - float(actual_deg) + 180.0) % 360.0) - 180.0


class DriveIO(Protocol):
    """Минимум, который нужен алгоритму от привода."""

    def read_angle(self) -> float: ...
    def set_speed(self, rpm: int) -> None: ...
    def run(self, *, reverse: bool) -> None: ...
    def hold(self) -> None: ...


@dataclass
class GotoProfile:
    """Настройки выхода в точку. Значения по умолчанию — осторожные."""

    tolerance_deg: float = 1.0
    """Допуск. Меньше ~0.5° механически недостижим: см. модуль-docstring."""

    max_rpm: int = 20
    min_rpm: int = 1

    coarse_deg: float = 20.0
    """Выше этой ошибки едем на максимальной скорости."""

    fine_deg: float = 3.0
    """Ниже этой ошибки переходим на толчки."""

    rpm_per_degree: float = 0.5
    """Наклон пропорционального участка между fine_deg и coarse_deg."""

    pulse_s: float = 0.05
    """Длительность одного толчка у цели."""

    settle_s: float = 0.25
    """Пауза после остановки перед контрольным чтением."""

    poll_s: float = 0.05
    """Период опроса на ходу."""

    max_attempts: int = 40
    timeout_s: float = 60.0

    stall_deg: float = 0.05
    """Насколько должен сместиться вал за проверку, чтобы не считаться залипшим."""

    stall_checks: int = 6
    """Сколько заходов подряд без движения считать заклиниванием."""

    def __post_init__(self) -> None:
        if self.tolerance_deg <= 0:
            raise ValueError("допуск должен быть положительным")
        if not 1 <= self.min_rpm <= self.max_rpm:
            raise ValueError("нужно 1 <= min_rpm <= max_rpm")
        if not 0 < self.fine_deg < self.coarse_deg:
            raise ValueError("нужно 0 < fine_deg < coarse_deg")


class GotoError(RuntimeError):
    """Не доехали."""


@dataclass
class GotoResult:
    reached: bool
    angle_deg: float
    error_deg: float
    attempts: int
    elapsed_s: float

    def __str__(self) -> str:
        verdict = "доехали" if self.reached else "НЕ доехали"
        return (f"{verdict}: {self.angle_deg:.2f}°, ошибка {self.error_deg:+.2f}°, "
                f"заходов {self.attempts}, {self.elapsed_s:.1f} с")


class GotoController:
    """Выход в угол с проверкой после остановки.

    ``clock`` и ``sleep`` вынесены наружу ради тестов: симулятор подставляет
    виртуальное время и прогоняет минуты движения за миллисекунды.
    """

    def __init__(self, drive: DriveIO, profile: GotoProfile | None = None, *,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep) -> None:
        self.drive = drive
        self.profile = profile or GotoProfile()
        self._clock = clock
        self._sleep = sleep

    def _speed_for(self, error: float) -> int:
        p = self.profile
        magnitude = abs(error)
        if magnitude >= p.coarse_deg:
            return p.max_rpm
        rpm = magnitude * p.rpm_per_degree
        return max(p.min_rpm, min(p.max_rpm, int(round(rpm))))

    def goto(self, target_deg: float, *,
             on_progress: Callable[[float, float], None] | None = None
             ) -> GotoResult:
        """Ехать в ``target_deg``, пока не окажемся внутри допуска."""
        p = self.profile
        target = normalize_360(target_deg)
        started = self._clock()
        attempts = 0
        stalled = 0
        previous_angle = None

        try:
            while True:
                angle = self.drive.read_angle()
                error = shortest_error(target, angle)
                if on_progress:
                    on_progress(angle, error)

                if abs(error) <= p.tolerance_deg:
                    return GotoResult(True, angle, error, attempts,
                                      self._clock() - started)

                if attempts >= p.max_attempts:
                    raise GotoError(
                        f"исчерпаны {p.max_attempts} заходов, остаточная ошибка "
                        f"{error:+.2f}° при допуске {p.tolerance_deg}°. "
                        f"Либо допуск слишком тугой для min_rpm={p.min_rpm}, "
                        f"либо вал что-то держит."
                    )
                if self._clock() - started > p.timeout_s:
                    raise GotoError(
                        f"таймаут {p.timeout_s} с, остаточная ошибка {error:+.2f}°"
                    )

                # Залипание: заходы идут, а вал не двигается.
                if previous_angle is not None:
                    moved = abs(shortest_error(angle, previous_angle))
                    stalled = stalled + 1 if moved < p.stall_deg else 0
                    if stalled >= p.stall_checks:
                        raise GotoError(
                            f"вал не двигается {stalled} заходов подряд на "
                            f"{angle:.2f}°. Проверьте разрешение привода (P-098), "
                            f"аварию (0x101A) и механику."
                        )
                previous_angle = angle

                attempts += 1
                if abs(error) <= p.fine_deg:
                    self._nudge(error)
                else:
                    self._approach(target, error)
        finally:
            # Останов важнее всего: выходим этим путём и по исключению, и по
            # успеху, и по Ctrl-C.
            self.drive.hold()

    def _approach(self, target: float, error: float) -> None:
        """Непрерывный ход, пока не войдём в зону толчков.

        Фаза намеренно ограничена по времени и прерывается, если вал перестал
        двигаться. Иначе при заклинившей или неразрешённой оси управление
        зависло бы здесь до общего таймаута, вместо того чтобы вернуться
        наверх и сказать, что именно не так.
        """
        p = self.profile
        self.drive.set_speed(self._speed_for(error))
        self.drive.run(reverse=error < 0)
        started_direction = error > 0

        # Сколько эта фаза может идти по-честному: путь на минимальной
        # скорости с запасом. Дольше — значит что-то не так.
        travel_s = abs(error) / (p.min_rpm * 6.0)
        deadline = self._clock() + min(p.timeout_s, travel_s * 1.5 + 1.0)

        previous = None
        still = 0
        while True:
            self._sleep(p.poll_s)
            angle = self.drive.read_angle()
            current = shortest_error(target, angle)

            # Вошли в зону точной доводки, либо проскочили цель насквозь.
            if abs(current) <= p.fine_deg or (current > 0) != started_direction:
                break
            if self._clock() > deadline:
                break

            if previous is not None:
                moved = abs(shortest_error(angle, previous))
                still = still + 1 if moved < p.stall_deg else 0
                if still >= p.stall_checks:
                    break          # наверх, там разберутся и сообщат
            previous = angle

            # Далеко -> ближе: снижаем скорость на ходу.
            self.drive.set_speed(self._speed_for(current))

        self.drive.hold()
        self._sleep(p.settle_s)

    def _nudge(self, error: float) -> None:
        """Один короткий толчок у цели, затем остановка и пауза."""
        p = self.profile
        self.drive.set_speed(p.min_rpm)
        self.drive.run(reverse=error < 0)
        self._sleep(p.pulse_s)
        self.drive.hold()
        self._sleep(p.settle_s)
