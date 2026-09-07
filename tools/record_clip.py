"""Записать кусок с камеры, чтобы потом мерить на нём распознавание.

Зачем: точность распознавания позы зависит от РАКУРСА. Сеть училась на
кадрах «человек напротив камеры», а у нас камера сверху, иногда под
углом. Спорить об этом словами бесполезно, а померить не на чем: чужие
записи сняты не с вашего места.

Запись делается ровно тем же путём, что и работа, — та же камера, тот же
цветной поток, — поэтому числа с неё переносятся на живой запуск.

    ./run_realsense.command --запись 10        # с RealSense, 10 секунд
    ./.venv/bin/python tools/record_clip.py --секунд 10 --камера 0

Кадры кладутся в clips/<дата>.npz. Один такой файл на 10 секунд занимает
десятки мегабайт, в гит они не идут.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

log = logging.getLogger("запись")


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--секунд", type=float, default=10.0)
    p.add_argument("--камера", type=int, default=0)
    p.add_argument("--realsense", action="store_true")
    p.add_argument("--куда", type=Path, default=None)
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    import numpy as np

    from arm_control import RealSenseFrames, looks_infrared, open_camera

    import cv2

    камера = (RealSenseFrames() if args.realsense
              else open_camera(cv2, args.камера))

    кадры = []
    предупредили = False
    начало = time.monotonic()
    try:
        while time.monotonic() - начало < args.секунд:
            ok, кадр = камера.read()
            if not ok:
                continue
            if not предупредили and looks_infrared(кадр):
                log.warning("кадр ИНФРАКРАСНЫЙ — запись будет серой, "
                            "и мерить по ней распознавание бессмысленно")
                предупредили = True
            кадры.append(кадр)
    finally:
        камера.release()

    if not кадры:
        log.error("камера не отдала ни одного кадра")
        return 1

    папка = Path(__file__).resolve().parents[1] / "clips"
    папка.mkdir(exist_ok=True)
    путь = args.куда or папка / time.strftime("клип_%Y%m%d_%H%M%S.npz")
    np.savez_compressed(путь, кадры=np.asarray(кадры))

    прошло = time.monotonic() - начало
    log.info("записано %d кадров за %.1f с (%.1f к/с) -> %s",
             len(кадры), прошло, len(кадры) / прошло, путь)
    log.info("теперь: ./.venv/bin/python tools/pose_compare.py %s", путь)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
