"""Подбор импульсного тракта сустава: редуктор, разрешение, потолок скорости.

    python examples/plan_axes.py --reduction 50 --encoder 17
    python examples/plan_axes.py --reduction 50 --ppr 3600 --sb1222
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import pulses as pu

CANDIDATES = (900, 1800, 3600, 7200, 14400, 10000)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", default="J1")
    p.add_argument("--reduction", type=float, required=True,
                   help="передаточное число редуктора сустава, i:1")
    p.add_argument("--encoder", type=int, default=17, choices=[17, 23],
                   help="разрядность энкодера")
    p.add_argument("--ppr", type=int, help="импульсов на оборот вала; "
                                           "без него печатается таблица вариантов")
    p.add_argument("--rated-rpm", type=float, default=3000.0)
    p.add_argument("--sb1222", action="store_true",
                   help="выход через SB 1222 (200 кГц вместо 100 кГц)")
    args = p.parse_args()

    counts = pu.ENCODER_17BIT if args.encoder == 17 else pu.ENCODER_23BIT
    max_hz = pu.PLC_MAX_HZ_SB1222 if args.sb1222 else pu.PLC_MAX_HZ_ONBOARD

    def plan(ppr: int) -> pu.JointPlan:
        return pu.JointPlan(
            name=args.name, reduction=args.reduction, pulses_per_rev=ppr,
            counts_per_rev=counts, plc_max_hz=max_hz,
            motor_rated_rpm=args.rated_rpm,
        )

    if args.ppr:
        print(plan(args.ppr).report())
        return 0

    print(f"энкодер {args.encoder} бит ({counts} имп./об), "
          f"редуктор 1:{args.reduction:g}, потолок ПЛК {max_hz/1000:g} кГц\n")
    header = f"{'имп./об':>8} {'град/имп':>10} {'имп./град':>10} " \
             f"{'об/мин вала':>12} {'град/с':>8}  эл. редуктор"
    print(header)
    print("-" * len(header))
    for ppr in sorted(CANDIDATES):
        try:
            j = plan(ppr)
            gear = f"{j.gear.numerator}/{j.gear.denominator}"
        except ValueError as exc:
            print(f"{ppr:>8}  -- {exc}")
            continue
        mark = "" if j.integer_pulses_per_degree else " *"
        print(f"{ppr:>8} {j.degrees_per_pulse:>10.5f} "
              f"{float(j.pulses_per_degree):>10.1f}{mark:<2} "
              f"{j.max_motor_rpm:>10.0f} {j.max_joint_deg_s:>8.0f}  {gear}")
    print("\n* дробное число импульсов на градус")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
