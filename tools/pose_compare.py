"""Сравнить модели позы и повороты кадра на ВАШЕЙ записи.

Три готовые модели (lite, full, heavy) — одна и та же сеть разной
величины. Какая лучше на камере сверху, из общих соображений не следует:
все они учились на кадрах «человек напротив камеры», и вид сверху для
всех непривычен. Померить можно только на записи с вашего места.

Считаются три вещи, и каждая отвечает на свой вопрос:

* ДОЛЯ КАДРОВ с найденными плечом, локтем и запястьем — видит ли модель
  руку вообще;
* УВЕРЕННОСТЬ, которую модель сама ставит точкам, — насколько твёрдо она
  их находит;
* ДРОЖЬ угла между соседними кадрами. Человек так быстро не двигается,
  поэтому дрожь — это шум, который потом придётся давить фильтром, а
  фильтр добавляет задержку.

    ./.venv/bin/python tools/pose_compare.py clips/клип_*.npz
"""

from __future__ import annotations

import argparse
import logging
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from scara_drive.armtrack import angles_from_points, wrap180  # noqa: E402

log = logging.getLogger("сравнение")

ВИДЫ = ("lite", "full", "heavy")
ПОВОРОТЫ = (0, 90, 180, 270)


def прогнать(кадры, вид: str, поворот: int, сторона: str):
    """Вернуть (доля кадров, средняя уверенность, дрожь, мс на кадр)."""
    import cv2
    import mediapipe as mp
    import numpy as np
    from mediapipe.tasks import python as tasks
    from mediapipe.tasks.python import vision

    from arm_control import POSE_POINTS

    путь = (Path(__file__).resolve().parents[1] / "models"
            / f"pose_landmarker_{вид}.task")
    if not путь.is_file():
        return None

    опции = vision.PoseLandmarkerOptions(
        base_options=tasks.BaseOptions(
            model_asset_path=str(путь),
            delegate=tasks.BaseOptions.Delegate.CPU),
        running_mode=vision.RunningMode.VIDEO, num_poses=1,
        min_pose_detection_confidence=0.5, min_tracking_confidence=0.5)
    сеть = vision.PoseLandmarker.create_from_options(опции)
    коды = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    нужны = POSE_POINTS[сторона]

    найдено = 0
    уверенности: list[float] = []
    углы: list[float] = []
    начало = time.monotonic()
    for номер, кадр in enumerate(кадры):
        кадр = cv2.flip(кадр, 1)
        if поворот:
            кадр = cv2.rotate(кадр, коды[поворот])
        rgb = cv2.cvtColor(кадр, cv2.COLOR_BGR2RGB)
        итог = сеть.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
            номер * 33)
        if not итог.pose_landmarks:
            углы.append(float("nan"))
            continue
        точки = итог.pose_landmarks[0]
        высота, ширина = rgb.shape[:2]
        собрано, видимости = {}, []
        плохо = False
        for роль, индекс in нужны.items():
            точка = точки[индекс]
            видимость = getattr(точка, "visibility", None)
            if видимость is not None:
                видимости.append(float(видимость))
                if видимость < 0.5:
                    плохо = True
            собрано[роль] = (точка.x * ширина, точка.y * высота)
        if плохо:
            углы.append(float("nan"))
            continue
        найдено += 1
        if видимости:
            уверенности.append(sum(видимости) / len(видимости))
        try:
            q1, _ = angles_from_points(собрано["shoulder"], собрано["elbow"],
                                       собрано["wrist"])
            углы.append(q1)
        except ValueError:
            углы.append(float("nan"))
    мс = (time.monotonic() - начало) / max(len(кадры), 1) * 1000
    сеть.close()

    скачки = [abs(wrap180(b - a)) for a, b in zip(углы, углы[1:])
              if a == a and b == b]
    дрожь = statistics.median(скачки) if скачки else float("nan")
    доля = найдено / max(len(кадры), 1)
    уверенность = sum(уверенности) / len(уверенности) if уверенности else 0.0
    return доля, уверенность, дрожь, мс


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("клип", type=Path)
    p.add_argument("--сторона", default="right", choices=["right", "left"])
    p.add_argument("--повороты", action="store_true",
                   help="перебрать ещё и повороты кадра")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    import numpy as np

    данные = np.load(args.клип)
    кадры = данные["кадры"]
    log.info("кадров в записи: %d, размер %dx%d\n",
             len(кадры), кадры.shape[2], кадры.shape[1])

    повороты = ПОВОРОТЫ if args.повороты else (0,)
    print(f"{'модель':7} {'поворот':>8} {'рука видна':>11} "
          f"{'уверенность':>12} {'дрожь угла':>11} {'мс/кадр':>8}")
    лучшее = None
    for вид in ВИДЫ:
        for поворот in повороты:
            итог = прогнать(кадры, вид, поворот, args.сторона)
            if итог is None:
                print(f"{вид:7} — нет файла модели")
                continue
            доля, увер, дрожь, мс = итог
            print(f"{вид:7} {поворот:>8} {доля*100:>10.0f}% "
                  f"{увер:>12.2f} {дрожь:>10.1f}° {мс:>8.1f}")
            оценка = (доля, -дрожь if дрожь == дрожь else -999)
            if лучшее is None or оценка > лучшее[0]:
                лучшее = (оценка, вид, поворот, мс)

    if лучшее:
        _, вид, поворот, мс = лучшее
        print(f"\nлучше всех держит руку: {вид}, поворот {поворот}, "
              f"{мс:.0f} мс на кадр")
        ключи = f"--pose-model {вид}"
        if поворот:
            ключи += f" --rotate {поворот}"
        print(f"запуск с ней:  {ключи}")
        print("если частота кадров при этом просядет — берите модель "
              "полегче: точность без частоты не помогает, рука убегает.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
