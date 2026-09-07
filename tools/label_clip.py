"""Разметить запись с камеры под обучение своей модели позы.

Зачем это вообще нужно. MediaPipe училась на кадрах «человек напротив
камеры». Камера сверху для неё — случай на отшибе, и никакой настройкой
это не лечится: модель просто не видела таких ракурсов. Единственный
надёжный путь — обучить свою на кадрах С ВАШЕГО МЕСТА.

Разметка — самая долгая часть, и она здесь удешевлена вдвое:

* Кадры сначала размечает тяжёлая MediaPipe. На части кадров она
  справляется даже сверху — эти кадры уже готовы, их нужно только
  подтвердить нажатием клавиши.
* Точек всего три: плечо, локоть, кисть. Это не 17 суставов, поэтому
  ручная правка кадра занимает секунды.

Точки ставятся ЩЕЛЧКАМИ в порядке плечо → локоть → кисть.

    ЛКМ            поставить очередную точку
    a              принять то, что предложила MediaPipe
    u              отменить последнюю точку
    n              следующий кадр (кадр без трёх точек не сохраняется)
    d              выбросить кадр совсем
    q              сохранить и выйти

Пример:

    ./.venv/bin/python tools/label_clip.py clips/клип_20260907.npz --шаг 5

На выходе — папка dataset/ в формате YOLO-pose: images/, labels/ и
data.yaml. Её можно скормить ultralytics как есть.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

log = logging.getLogger("разметка")

ТОЧКИ = ("плечо", "локоть", "кисть")
ЦВЕТА = ((255, 170, 60), (60, 220, 130), (80, 80, 240))   # BGR

#: Запас вокруг точек для рамки, в долях от размаха руки. YOLO-pose
#: требует рамку, а рука — не объект с очевидными границами: берём
#: описанный прямоугольник и раздвигаем, чтобы плечо и кисть не лежали
#: ровно на краю.
ЗАПАС = 0.25


def предразметка(кадры, вид: str = "heavy", сторона: str = "right"):
    """Прогнать модель по всем кадрам. Вернуть список точек или None."""
    import cv2
    import mediapipe as mp
    from mediapipe.tasks import python as tasks
    from mediapipe.tasks.python import vision

    from arm_control import POSE_POINTS

    путь = (Path(__file__).resolve().parents[1] / "models"
            / f"pose_landmarker_{вид}.task")
    if not путь.is_file():
        log.warning("нет модели %s — размечать придётся всё вручную", путь.name)
        return [None] * len(кадры)

    опции = vision.PoseLandmarkerOptions(
        base_options=tasks.BaseOptions(
            model_asset_path=str(путь),
            delegate=tasks.BaseOptions.Delegate.CPU),
        running_mode=vision.RunningMode.VIDEO, num_poses=1,
        min_pose_detection_confidence=0.4, min_tracking_confidence=0.4)
    сеть = vision.PoseLandmarker.create_from_options(опции)
    нужны = POSE_POINTS[сторона]

    готово = []
    for номер, кадр in enumerate(кадры):
        rgb = cv2.cvtColor(кадр, cv2.COLOR_BGR2RGB)
        итог = сеть.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), номер * 33)
        if not итог.pose_landmarks:
            готово.append(None)
            continue
        точки = итог.pose_landmarks[0]
        высота, ширина = кадр.shape[:2]
        собрано = []
        плохо = False
        for роль in ("shoulder", "elbow", "wrist"):
            точка = точки[нужны[роль]]
            видимость = getattr(точка, "visibility", None)
            if видимость is not None and видимость < 0.4:
                плохо = True
            собрано.append((точка.x * ширина, точка.y * высота))
        готово.append(None if плохо else собрано)
    сеть.close()
    return готово


def в_строку_yolo(точки, ширина: int, высота: int) -> str:
    """Одна строка разметки YOLO-pose: класс, рамка и три точки.

    Всё в долях от размера кадра — так требует формат, и так разметка
    переживает смену разрешения камеры.

    Размах руки тут 80 пикселей, запас четверть — по 20 в каждую сторону.
    Рамка выходит 120x80 с центром в (60, 60), то есть 0.6 x 0.4 от кадра.

    >>> в_строку_yolo([(20, 40), (60, 80), (100, 40)], 200, 200)
    '0 0.300000 0.300000 0.600000 0.400000 0.100000 0.200000 2 0.300000 0.400000 2 0.500000 0.200000 2'

    Точки у края кадра рамку за край не выводят:

    >>> в_строку_yolo([(0, 0), (10, 10), (20, 0)], 100, 100)[:34]
    '0 0.125000 0.075000 0.250000 0.150'
    """
    иксы = [т[0] for т in точки]
    игреки = [т[1] for т in точки]
    размах = max(max(иксы) - min(иксы), max(игреки) - min(игреки), 1.0)
    запас = размах * ЗАПАС

    лево = max(0.0, min(иксы) - запас)
    верх = max(0.0, min(игреки) - запас)
    право = min(float(ширина), max(иксы) + запас)
    низ = min(float(высота), max(игреки) + запас)

    цx = (лево + право) / 2 / ширина
    цy = (верх + низ) / 2 / высота
    ш = (право - лево) / ширина
    в = (низ - верх) / высота

    куски = [f"0 {цx:.6f} {цy:.6f} {ш:.6f} {в:.6f}"]
    for x, y in точки:
        куски.append(f"{x / ширина:.6f} {y / высота:.6f} 2")
    return " ".join(куски)


def нарисовать(кадр, точки, подсказка: str):
    """Кадр с уже поставленными точками и строкой состояния."""
    import cv2

    холст = кадр.copy()
    for номер, точка in enumerate(точки):
        x, y = int(точка[0]), int(точка[1])
        cv2.circle(холст, (x, y), 6, ЦВЕТА[номер], -1)
        cv2.putText(холст, ТОЧКИ[номер], (x + 10, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, ЦВЕТА[номер], 1,
                    cv2.LINE_AA)
    for a, b in ((0, 1), (1, 2)):
        if len(точки) > b:
            cv2.line(холст, tuple(map(int, точки[a])),
                     tuple(map(int, точки[b])), (200, 200, 200), 2)

    высота = холст.shape[0]
    cv2.rectangle(холст, (0, высота - 28), (холст.shape[1], высота),
                  (30, 30, 30), -1)
    cv2.putText(холст, подсказка, (8, высота - 9),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)
    return холст


def сохранить(куда: Path, кадры, разметка, доля_проверки: float = 0.2) -> int:
    """Разложить кадры и подписи в формате YOLO-pose, с разделением.

    Обучающая и проверочная части РАЗНЫЕ намеренно. Проверять на тех же
    кадрах, на которых учились, — значит получить красивую цифру, которая
    ничего не значит: сеть их уже видела. Разделение делает первую же
    оценку честной.

    Кадры идут подряд из одной записи, и соседние почти одинаковы.
    Поэтому в проверку уходит ХВОСТ записи, а не случайные кадры:
    случайная выборка растащила бы почти одинаковые кадры по обе
    стороны, и проверка снова оказалась бы поддавками.

    Имена файлов латиницей: датасет уходит в чужие руки — ultralytics,
    и рисковать на кириллице незачем.
    """
    import cv2

    номера = sorted(разметка)
    граница = max(1, int(len(номера) * (1 - доля_проверки)))
    части = {"train": номера[:граница], "val": номера[граница:]}
    if not части["val"]:                 # кадров совсем мало
        части["val"] = части["train"][-1:]

    сохранено = 0
    for часть, список in части.items():
        картинки = куда / "images" / часть
        подписи = куда / "labels" / часть
        картинки.mkdir(parents=True, exist_ok=True)
        подписи.mkdir(parents=True, exist_ok=True)
        for номер in список:
            кадр = кадры[номер]
            имя = f"frame_{номер:05d}"
            cv2.imwrite(str(картинки / f"{имя}.jpg"), кадр,
                        [cv2.IMWRITE_JPEG_QUALITY, 92])
            высота, ширина = кадр.shape[:2]
            (подписи / f"{имя}.txt").write_text(
                в_строку_yolo(разметка[номер], ширина, высота) + "\n",
                encoding="utf-8")
            сохранено += 1

    (куда / "data.yaml").write_text(
        f"path: {куда.resolve()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "kpt_shape: [3, 3]\n"
        "flip_idx: [0, 1, 2]\n"
        "names:\n"
        "  0: arm\n", encoding="utf-8")
    log.info("обучающих кадров %d, проверочных %d",
             len(части["train"]), len(части["val"]))
    return сохранено


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("клип", type=Path)
    p.add_argument("--шаг", type=int, default=5,
                   help="брать каждый N-й кадр: соседние кадры почти "
                        "одинаковы, и размечать их все — трата времени")
    p.add_argument("--сторона", default="right", choices=["right", "left"])
    p.add_argument("--модель", default="heavy",
                   choices=["lite", "full", "heavy"],
                   help="чем предразмечать")
    p.add_argument("--куда", type=Path, default=None)
    p.add_argument("--без-окна", action="store_true",
                   help="только предразметка, без ручной правки")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    import numpy as np

    данные = np.load(args.клип)
    все_кадры = данные["кадры"]
    номера = list(range(0, len(все_кадры), max(1, args.шаг)))
    кадры = {н: все_кадры[н] for н in номера}
    log.info("кадров в записи %d, размечаем каждый %d-й — %d штук",
             len(все_кадры), args.шаг, len(номера))

    log.info("предразметка моделью %s...", args.модель)
    подсказки_список = предразметка([все_кадры[н] for н in номера],
                                    args.модель, args.сторона)
    подсказки = dict(zip(номера, подсказки_список))
    нашлось = sum(1 for т in подсказки.values() if т)
    log.info("модель справилась сама с %d из %d кадров (%.0f%%)",
             нашлось, len(номера), нашлось / max(len(номера), 1) * 100)

    куда = args.куда or (Path(__file__).resolve().parents[1] / "dataset")

    if args.без_окна:
        разметка = {н: т for н, т in подсказки.items() if т}
        сколько = сохранить(куда, кадры, разметка)
        log.info("сохранено %d кадров в %s (только автоматические)",
                 сколько, куда)
        return 0

    import cv2

    разметка: dict[int, list] = {}
    окно = "разметка: ЛКМ ставит точку, a принять, u отменить, n дальше, q выход"
    cv2.namedWindow(окно, cv2.WINDOW_NORMAL)
    текущие: list = []

    def щелчок(событие, x, y, флаги, _):
        if событие == cv2.EVENT_LBUTTONDOWN and len(текущие) < 3:
            текущие.append((float(x), float(y)))

    cv2.setMouseCallback(окно, щелчок)

    место = 0
    while место < len(номера):
        номер = номера[место]
        кадр = кадры[номер]
        подсказка_точки = подсказки.get(номер)

        while True:
            если_есть = " | a — принять подсказку" if подсказка_точки else ""
            строка = (f"{место + 1}/{len(номера)}  "
                      f"дальше: {ТОЧКИ[len(текущие)] if len(текущие) < 3 else 'готово, n'}"
                      f"{если_есть}")
            показ = кадр
            if not текущие and подсказка_точки:
                показ = нарисовать(кадр, подсказка_точки,
                                   строка + "  [подсказка модели]")
            else:
                показ = нарисовать(кадр, текущие, строка)
            cv2.imshow(окно, показ)

            клавиша = cv2.waitKey(20) & 0xFF
            if клавиша == ord("q"):
                место = len(номера)
                break
            if клавиша == ord("a") and подсказка_точки:
                текущие = list(подсказка_точки)
            elif клавиша == ord("u") and текущие:
                текущие.pop()
            elif клавиша == ord("d"):
                текущие = []
                место += 1
                break
            elif клавиша == ord("n"):
                if len(текущие) == 3:
                    разметка[номер] = list(текущие)
                текущие = []
                место += 1
                break

    cv2.destroyAllWindows()

    if not разметка:
        log.info("ничего не размечено — файлы не тронуты")
        return 0

    сколько = сохранить(куда, кадры, разметка)
    log.info("сохранено %d кадров в %s", сколько, куда)
    log.info("дальше:")
    log.info("  pip install ultralytics")
    log.info("  yolo pose train data=%s model=yolo11n-pose.pt "
             "epochs=100 imgsz=640", куда / "data.yaml")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
