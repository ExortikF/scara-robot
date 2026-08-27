"""Бюджет времени супервизорного контура PID поверх Modbus RTU.

Отвечает на вопрос «почему ось не встаёт в допуск»: считает, сколько времени
занимает один проход контура и сколько градусов вал успевает пройти за это
время. Если проход больше допуска — контур не может сойтись в принципе, он
может только рыскать вокруг цели.

    python tools/pid_budget.py --baud 19200 --parity E --min-rpm 1 --tolerance 0.5
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass


def bits_per_byte(parity: str, stopbits: int) -> int:
    """Старт + данные + чётность + стоп."""
    return 1 + 8 + (0 if parity.upper() == "N" else 1) + stopbits


@dataclass
class Budget:
    baud: int
    parity: str
    stopbits: int
    inter_frame_delay: float   # с, пауза между транзакциями в программе
    turnaround: float          # с, задержка ответа привода
    loop_sleep: float          # с, sleep в конце прохода

    def frame_s(self, nbytes: int) -> float:
        return nbytes * bits_per_byte(self.parity, self.stopbits) / self.baud

    def transaction_s(self, request_bytes: int, response_bytes: int) -> float:
        return (self.inter_frame_delay
                + self.frame_s(request_bytes)
                + self.turnaround
                + self.frame_s(response_bytes))

    def read_s(self, registers: int) -> float:
        # FC03: запрос 8 байт, ответ 5 + 2*N
        return self.transaction_s(8, 5 + 2 * registers)

    def write_s(self) -> float:
        # FC06: запрос 8 байт, ответ 8 байт (эхо)
        return self.transaction_s(8, 8)

    def loop_s(self, *, reads: int, writes: int, read_registers: int = 2) -> float:
        return (reads * self.read_s(read_registers)
                + writes * self.write_s()
                + self.loop_sleep)


def report(budget: Budget, *, min_rpm: float, max_rpm: float,
           tolerance_deg: float) -> None:
    # Как устроен проход в c30_servo_gui: чтение положения (2 регистра),
    # чтение фактической скорости (1 регистр), иногда запись скорости.
    quiet = budget.loop_s(reads=2, writes=0)
    moving = budget.loop_s(reads=2, writes=1)
    turning = budget.loop_s(reads=2, writes=3) + 0.04  # разворот: HOLD + пауза

    print(f"порт {budget.baud} 8{budget.parity}{budget.stopbits}, "
          f"пауза между кадрами {budget.inter_frame_delay*1000:.0f} мс, "
          f"sleep в конце {budget.loop_sleep*1000:.0f} мс\n")

    print("время одного прохода контура:")
    for name, value in (("удержание (2 чтения)", quiet),
                        ("движение (+запись скорости)", moving),
                        ("смена направления", turning)):
        print(f"  {name:<30} {value*1000:6.0f} мс   ({1/value:4.1f} Гц)")

    print(f"\nпуть вала за один проход (движение, {moving*1000:.0f} мс):")
    for rpm in sorted({min_rpm, 2.0, 5.0, max_rpm}):
        deg = rpm * 6.0 * moving          # 1 об/мин = 6 град/с
        flag = "  <-- меньше допуска" if deg < tolerance_deg else ""
        print(f"  {rpm:5.1f} об/мин = {rpm*6:6.1f} град/с -> {deg:7.2f}°{flag}")

    floor = min_rpm * 6.0 * moving
    print(f"\nдопуск задан:                {tolerance_deg:.2f}°")
    print(f"минимальный шаг контура:     {floor:.2f}° "
          f"(на {min_rpm:g} об/мин — медленнее привод не поедет)")

    if floor >= tolerance_deg:
        need_rpm = tolerance_deg / (2 * 6.0 * moving)
        print("\nВЕРДИКТ: контур сойтись не может.")
        print(f"  За один проход вал проходит {floor:.2f}°, а встать надо "
              f"в {tolerance_deg:.2f}°.")
        print("  Контур физически не видит ничего мельче своего шага, поэтому")
        print("  проскакивает цель, разворачивается, проскакивает обратно.")
        print(f"\n  Чтобы допуск {tolerance_deg:.2f}° стал достижим, нужна скорость")
        print(f"  подхода {need_rpm:.2f} об/мин — но привод принимает целые")
        print(f"  об/мин, и минимум это 1 об/мин = {6.0*moving:.2f}° за проход.")
        print(f"\n  Значит при текущем цикле реально достижимый допуск — "
              f"около {2*floor:.1f}°.")
    else:
        print("\nВЕРДИКТ: допуск достижим.")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--parity", default="E", choices=["N", "E", "O"])
    p.add_argument("--stopbits", type=int, default=1)
    p.add_argument("--gap", type=float, default=0.030, help="inter_frame_delay, с")
    p.add_argument("--turnaround", type=float, default=0.005,
                   help="задержка ответа привода, с")
    p.add_argument("--sleep", type=float, default=0.060, help="sleep в конце прохода, с")
    p.add_argument("--min-rpm", type=float, default=1.0)
    p.add_argument("--max-rpm", type=float, default=20.0)
    p.add_argument("--tolerance", type=float, default=0.5)
    args = p.parse_args()

    budget = Budget(baud=args.baud, parity=args.parity, stopbits=args.stopbits,
                    inter_frame_delay=args.gap, turnaround=args.turnaround,
                    loop_sleep=args.sleep)
    report(budget, min_rpm=args.min_rpm, max_rpm=args.max_rpm,
           tolerance_deg=args.tolerance)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
