"""Проверка выхода в угол на симуляторе привода.

Симулятор воспроизводит то, что мешает регулятору на реальном железе:

* задание скорости целое, в об/мин — дробную скорость привод не примет;
* скорость нарастает не мгновенно (первый порядок);
* каждая транзакция Modbus отъедает время;
* чтение положения возвращает данные с задержкой.

Время виртуальное, поэтому минуты движения проходят за миллисекунды.
"""

from __future__ import annotations

import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive.goto import (
    GotoController, GotoError, GotoProfile, normalize_360, shortest_error,
)


class SimulatedDrive:
    """Модель привода в скоростном режиме с обменом по RS-485."""

    def __init__(self, *, angle=0.0, txn_s=0.008, read_latency_s=0.03,
                 tau_s=0.08, jitter_deg=0.0, seed=0):
        self.angle = float(angle)
        self.speed_deg_s = 0.0
        self.commanded_deg_s = 0.0
        self.txn_s = txn_s              # длительность одной транзакции
        self.read_latency_s = read_latency_s
        self.tau_s = tau_s              # постоянная времени разгона
        self.jitter_deg = jitter_deg    # шум датчика
        self._rpm = 0
        self._reverse = False
        self._running = False
        self._t = 0.0
        self._history = [(0.0, self.angle)]
        self._random = random.Random(seed)

    # --- виртуальное время ---
    def clock(self) -> float:
        return self._t

    def sleep(self, seconds: float) -> None:
        self._advance(seconds)

    def _advance(self, seconds: float) -> None:
        step = 0.002
        remaining = seconds
        while remaining > 0:
            dt = min(step, remaining)
            target = self.commanded_deg_s if self._running else 0.0
            self.speed_deg_s += (target - self.speed_deg_s) * (dt / self.tau_s)
            self.angle = normalize_360(self.angle + self.speed_deg_s * dt)
            self._t += dt
            self._history.append((self._t, self.angle))
            remaining -= dt

    def _transaction(self) -> None:
        self._advance(self.txn_s)

    # --- интерфейс DriveIO ---
    def read_angle(self) -> float:
        self._transaction()
        wanted = self._t - self.read_latency_s
        angle = self._history[0][1]
        for stamp, value in self._history:
            if stamp > wanted:
                break
            angle = value
        if self.jitter_deg:
            angle += self._random.uniform(-self.jitter_deg, self.jitter_deg)
        return normalize_360(angle)

    def set_speed(self, rpm: int) -> None:
        assert isinstance(rpm, int) and rpm >= 1, f"привод примет только целое >=1, дали {rpm!r}"
        self._transaction()
        self._rpm = rpm
        self._apply()

    def run(self, *, reverse: bool) -> None:
        self._transaction()
        self._reverse = reverse
        self._running = True
        self._apply()

    def hold(self) -> None:
        self._transaction()
        self._running = False
        self._apply()

    def _apply(self) -> None:
        speed = self._rpm * 6.0        # об/мин -> град/с
        self.commanded_deg_s = -speed if self._reverse else speed


class StuckDrive(SimulatedDrive):
    """Вал заклинило: команды принимает, но не двигается."""

    def _apply(self) -> None:
        self.commanded_deg_s = 0.0


class TestAngleMath(unittest.TestCase):
    def test_shortest_error_crosses_zero(self):
        self.assertAlmostEqual(shortest_error(1, 359), 2.0)
        self.assertAlmostEqual(shortest_error(359, 1), -2.0)

    def test_normalize(self):
        self.assertAlmostEqual(normalize_360(-0.5), 359.5)
        self.assertAlmostEqual(normalize_360(720.25), 0.25)


class TestGoto(unittest.TestCase):
    def _run(self, start, target, **kwargs):
        drive = SimulatedDrive(angle=start, **kwargs)
        controller = GotoController(
            drive, GotoProfile(), clock=drive.clock, sleep=drive.sleep,
        )
        return drive, controller.goto(target)

    def test_arrives_from_far(self):
        _, result = self._run(0.0, 180.0)
        self.assertTrue(result.reached, str(result))
        self.assertLessEqual(abs(result.error_deg), 1.0)

    def test_takes_short_way_across_zero(self):
        drive, result = self._run(359.0, 1.0)
        self.assertTrue(result.reached, str(result))
        # Кратчайший путь — вперёд на 2°, а не назад на 358°.
        self.assertLess(result.elapsed_s, 5.0, "поехал длинной дорогой")

    def test_already_there_does_not_move(self):
        drive, result = self._run(90.0, 90.0)
        self.assertTrue(result.reached)
        self.assertEqual(result.attempts, 0)

    def test_tiny_move(self):
        _, result = self._run(90.0, 92.0)
        self.assertTrue(result.reached, str(result))

    def test_survives_sensor_noise(self):
        _, result = self._run(0.0, 137.0, jitter_deg=0.2)
        self.assertTrue(result.reached, str(result))

    def test_survives_slow_link(self):
        # Вдвое медленнее обмен и вдвое инертнее вал.
        _, result = self._run(10.0, 200.0, txn_s=0.02,
                              read_latency_s=0.08, tau_s=0.2)
        self.assertTrue(result.reached, str(result))

    def test_reports_stall_instead_of_spinning_forever(self):
        drive = StuckDrive(angle=0.0)
        controller = GotoController(
            drive, GotoProfile(), clock=drive.clock, sleep=drive.sleep,
        )
        with self.assertRaises(GotoError) as ctx:
            controller.goto(90.0)
        self.assertIn("не двигается", str(ctx.exception))

    def test_stops_the_drive_on_failure(self):
        drive = StuckDrive(angle=0.0)
        controller = GotoController(
            drive, GotoProfile(), clock=drive.clock, sleep=drive.sleep,
        )
        with self.assertRaises(GotoError):
            controller.goto(90.0)
        self.assertFalse(drive._running, "привод остался запущенным после отказа")

    def test_converges_from_many_starts(self):
        """Главная проверка: доезжает откуда угодно куда угодно."""
        rng = random.Random(20260827)
        worst = 0.0
        slowest = 0.0
        for _ in range(60):
            start = rng.uniform(0, 360)
            target = rng.uniform(0, 360)
            drive, result = self._run(start, target, seed=rng.randint(0, 9999))
            self.assertTrue(
                result.reached,
                f"не доехал {start:.1f}° -> {target:.1f}°: {result}",
            )
            worst = max(worst, abs(result.error_deg))
            slowest = max(slowest, result.elapsed_s)
        print(f"\n  60 прогонов: худшая ошибка {worst:.2f}°, "
              f"дольше всего {slowest:.1f} с")
        self.assertLessEqual(worst, 1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
