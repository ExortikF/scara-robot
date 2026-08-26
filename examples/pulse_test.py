"""Первый прогон импульсов: план разгона и отправка в МК.

Посмотреть план, ничего не подключая:

    python examples/pulse_test.py --dry-run --degrees 90

Прогнать на железе (мотор без нагрузки!):

    python examples/pulse_test.py --port /dev/tty.usbmodem1101 --degrees 90

Проверка «туда-обратно» — вал обязан вернуться на то же место:

    python examples/pulse_test.py --port /dev/tty.usbmodem1101 --degrees 90 --there-and-back
"""

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import pulses as pu
from scara_drive.stepper_link import RampProfile, StepperLink, describe, plan_ramp


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="последовательный порт МК")
    p.add_argument("--dry-run", action="store_true", help="только напечатать план")
    p.add_argument("--axis", type=int, default=0)
    p.add_argument("--degrees", type=float, default=90.0, help="поворот сустава")
    p.add_argument("--reduction", type=float, default=50.0)
    p.add_argument("--ppr", type=int, default=3600, help="импульсов на оборот вала")
    p.add_argument("--encoder", type=int, default=17, choices=[17, 23])
    p.add_argument("--max-hz", type=int, default=20_000, help="пик частоты")
    p.add_argument("--accel", type=float, default=200_000.0, help="имп./с²")
    p.add_argument("--there-and-back", action="store_true")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    if not args.dry_run and not args.port:
        p.error("нужен --port или --dry-run")

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(message)s")

    joint = pu.JointPlan(
        name=f"J{args.axis}", reduction=args.reduction, pulses_per_rev=args.ppr,
        counts_per_rev=pu.ENCODER_17BIT if args.encoder == 17 else pu.ENCODER_23BIT,
    )
    steps = joint.pulses_for(args.degrees)
    print(joint.report())
    print(f"\nповорот на {args.degrees:g}° = {steps} импульсов\n")

    profile = RampProfile(max_step_hz=args.max_hz, accel_steps_s2=args.accel)

    if args.dry_run:
        print(describe(plan_ramp(steps, profile)))
        if args.there_and_back:
            print("\nобратный ход — зеркально")
        return 0

    with StepperLink(args.port) as link:
        print(link.version())
        link.move(args.axis, steps, profile)
        if args.there_and_back:
            link.move(args.axis, -steps, profile)
        final = link.positions()
        print(f"положение по счётчику МК: {final}")
        if args.there_and_back:
            drift = final.get(args.axis, 0)
            print(f"после «туда-обратно» должно быть 0, получено {drift}")
            print("расхождение по счётчику МК означает ошибку в отправке; "
                  "если счётчик 0, а вал сместился — теряются импульсы "
                  "на входе привода")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
