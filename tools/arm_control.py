"""Управление двумя осями SCARA по позе вытянутой руки. Камера сверху.

Камера смотрит на руку сверху, поэтому рука движется в той же плоскости, в
которой работает SCARA. Отсюда прямое соответствие без всякого пересчёта:

    плечо человека  ->  ось J1 (первое звено)
    локоть человека ->  ось J2 (второе звено)

Углы берутся из векторов между точками, а не из их положения в кадре, поэтому
не зависят ни от того, где человек стоит, ни от расстояния до камеры. Подробно
в ``src/scara_drive/armtrack.py``.

Источники точек — на выбор, потому что надёжность у них разная:

  pose     плечо/локоть/запястье распознаёт mediapipe Pose. Ставить ничего не
           надо, но вид сверху для него нетипичен: модель училась на съёмке
           человека спереди, и сверху может ошибаться.

  markers  три цветных метки на плече, локте и запястье, поиск по цвету в
           OpenCV. Нужны наклейки или цветная лента, зато распознавание не
           зависит от ракурса и работает стабильно.

Начните с ``pose``. Если точки скачут — переходите на ``markers``, это не
запасной вариант «на крайний случай», а решение понадёжнее.

Запуск:

    python tools/arm_control.py --simulate            без приводов
    python tools/arm_control.py --confirm             на живых приводах
    python tools/arm_control.py --confirm --source markers

ВАЖНО: с --confirm приводы КРУТЯТСЯ. Оси разгрузить, аварийный останов
держать под рукой. В окне есть кнопки ПАУЗА и СТОП.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import DriveBus, SerialConfig, TcpConfig
from scara_drive.armtrack import ArmFilter, angles_from_points, wrap180
from scara_drive.discovery import guess_port
from scara_drive.follow import AngleFollower, FollowConfig, MultiAxisFollower

log = logging.getLogger("arm_control")

AXES_FILE = Path(__file__).with_name("arm_axes.json")

class Funnel:
    """Счётчики по стадиям обработки кадра.

    Когда «рука видна, а робот не реагирует», важно знать, на каком именно
    шаге теряются кадры: камера не отдала, позу не нашли, точка не видна,
    геометрия не сошлась или фильтр отбросил. Без этого причину приходится
    угадывать.
    """

    STAGES = ("кадров", "поза найдена", "точки видны", "углы посчитаны",
              "фильтр принял")

    def __init__(self):
        self.counts = dict.fromkeys(self.STAGES, 0)
        self.last_loss = ""

    def hit(self, stage: str) -> None:
        self.counts[stage] += 1

    def lost(self, reason: str) -> None:
        self.last_loss = reason

    def report(self) -> str:
        frames = self.counts["кадров"] or 1
        parts = [f"{name} {self.counts[name]} ({self.counts[name] / frames * 100:.0f}%)"
                 for name in self.STAGES]
        return "   ".join(parts)


# Точки скелета mediapipe Pose.
POSE_POINTS = {
    "left":  {"shoulder": 11, "elbow": 13, "wrist": 15},
    "right": {"shoulder": 12, "elbow": 14, "wrist": 16},
}


@dataclass
class ArmAxis:
    """Ось робота и её связь с суставом человека."""

    name: str
    device_id: int
    joint: str            # "q1" — плечо, "q2" — локоть
    span_deg: float       # полный допустимый ход относительно нуля
    gain: float = 1.0     # во сколько раз повторять движение человека
    invert: bool = False

    def target_deg(self, joint_deg: float) -> float:
        """Угол сустава человека -> смещение оси от её нуля."""
        value = joint_deg * self.gain
        if self.invert:
            value = -value
        half = self.span_deg / 2
        return max(-half, min(value, half))


DEFAULT_AXES = [
    ArmAxis("J1 плечо",  1, "q1", span_deg=120.0),
    ArmAxis("J2 локоть", 2, "q2", span_deg=120.0),
]


def load_axes() -> list[ArmAxis]:
    if not AXES_FILE.is_file():
        AXES_FILE.write_text(
            json.dumps([asdict(a) for a in DEFAULT_AXES], ensure_ascii=False,
                       indent=2), encoding="utf-8")
        log.info("создан %s — пределы осей правьте там", AXES_FILE.name)
        return DEFAULT_AXES
    return [ArmAxis(**item) for item in
            json.loads(AXES_FILE.read_text(encoding="utf-8"))]


class SimulatedAxis:
    """Ось на бумаге: интегрирует заданную скорость."""

    def __init__(self, counts_per_rev: int = 131072, invert: bool = True):
        self.counts_per_rev, self.invert = counts_per_rev, invert
        self.position, self.rpm = 0, 0.0
        self._last = time.monotonic()

    def _advance(self):
        now = time.monotonic()
        dt, self._last = now - self._last, now
        counts = self.rpm / 60.0 * self.counts_per_rev * dt
        self.position += int(-counts if self.invert else counts)

    def read_position(self):
        time.sleep(0.006); self._advance(); return self.position

    def enable(self): pass
    def hold(self): self._advance(); self.rpm = 0.0
    def stop(self): self.hold()

    def run(self, speed, *, reverse=False):
        time.sleep(0.006); self._advance()
        self.rpm = -speed if reverse else speed


class PoseArmSource:
    """Плечо, локоть и запястье с mediapipe Pose."""

    name = "pose"
    MODEL_NAME = "pose_landmarker_full.task"
    MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
                 "pose_landmarker_full/float16/1/pose_landmarker_full.task")

    #: Ниже этого mediapipe сам не уверен в точке — такой кадр не берём.
    MIN_VISIBILITY = 0.5

    def __init__(self, camera_index: int = 0, side: str = "right",
                 model_path: Path | None = None):
        try:
            import cv2
            import mediapipe as mp
            from mediapipe.tasks import python as mp_tasks
            from mediapipe.tasks.python import vision
        except ImportError as exc:
            raise SystemExit(f"нужны mediapipe и opencv (не найдено: {exc.name})")

        self._cv2, self._mp = cv2, mp
        self.side = side
        self.points = POSE_POINTS[side]

        model = model_path or (Path(__file__).resolve().parents[1]
                               / "models" / self.MODEL_NAME)
        if not model.is_file():
            raise SystemExit(
                f"нет файла модели: {model}\n\n"
                "Скачайте один раз (около 6 МБ):\n"
                f"    mkdir -p {model.parent}\n"
                f"    curl -L -o {model} \\\n        {self.MODEL_URL}\n"
            )

        options = vision.PoseLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(
                model_asset_path=str(model),
                delegate=mp_tasks.BaseOptions.Delegate.CPU),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._landmarker = vision.PoseLandmarker.create_from_options(options)
        self._capture = open_camera(cv2, camera_index)
        self._start = time.monotonic()
        self.last_frame = None
        self.last_points = None
        self.note = ""
        self.funnel = Funnel()

    def poll(self):
        """Вернуть ((плечо), (локоть), (запястье)) в пикселях или None."""
        ok, frame = self._capture.read()
        if not ok:
            self.note = "камера не отдала кадр"
            self.funnel.lost(self.note)
            return None
        self.funnel.hit("кадров")

        frame = self._cv2.flip(frame, 1)
        rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(
            image, int((time.monotonic() - self._start) * 1000))

        self.last_frame = rgb
        if not result.pose_landmarks:
            self.note = "человека в кадре нет"
            self.funnel.lost(self.note)
            self.last_points = None
            return None
        self.funnel.hit("поза найдена")

        landmarks = result.pose_landmarks[0]
        height, width = rgb.shape[:2]
        found = {}
        for role, index in self.points.items():
            point = landmarks[index]
            # visibility у Tasks API — float ИЛИ None, если модель его не
            # выдала. Сравнивать None с числом нельзя: это исключение, а не
            # отбраковка, и оно раньше насмерть убивало цикл опроса.
            visibility = getattr(point, "visibility", None)
            if visibility is not None and visibility < self.MIN_VISIBILITY:
                self.note = f"{role}: точка не видна ({visibility:.2f})"
                self.funnel.lost(self.note)
                self.last_points = None
                return None
            found[role] = (point.x * width, point.y * height)
        self.funnel.hit("точки видны")

        self.note = ""
        self.last_points = (found["shoulder"], found["elbow"], found["wrist"])
        return self.last_points

    def close(self):
        self._capture.release()
        self._landmarker.close()


class MarkerArmSource:
    """Три цветные метки: плечо, локоть, запястье. Поиск по цвету.

    Не зависит от ракурса, поэтому для съёмки сверху надёжнее, чем модель позы.
    Диапазоны цветов в HSV лежат в ``arm_markers.json`` рядом со скриптом и
    правятся под вашу ленту.
    """

    name = "markers"
    CONFIG = Path(__file__).with_name("arm_markers.json")

    #: OpenCV держит оттенок в диапазоне 0..179, а не 0..359.
    DEFAULTS = {
        "shoulder": {"lower": [100, 120, 70], "upper": [130, 255, 255]},  # синий
        "elbow":    {"lower": [40, 90, 70],   "upper": [85, 255, 255]},   # зелёный
        "wrist":    {"lower": [140, 110, 70], "upper": [175, 255, 255]},  # розовый
    }
    MIN_AREA_PX = 60

    def __init__(self, camera_index: int = 0):
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise SystemExit(f"нужен opencv (не найдено: {exc.name})")

        self._cv2, self._np = cv2, np
        if not self.CONFIG.is_file():
            self.CONFIG.write_text(
                json.dumps(self.DEFAULTS, ensure_ascii=False, indent=2),
                encoding="utf-8")
            log.info("создан %s — цвета меток правьте там", self.CONFIG.name)
        self.ranges = json.loads(self.CONFIG.read_text(encoding="utf-8"))

        self._capture = open_camera(cv2, camera_index)
        self.last_frame = None
        self.last_points = None
        self.note = ""
        self.funnel = Funnel()

    def _find(self, hsv, spec):
        cv2, np = self._cv2, self._np
        mask = cv2.inRange(hsv, np.array(spec["lower"], np.uint8),
                           np.array(spec["upper"], np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        biggest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest) < self.MIN_AREA_PX:
            return None
        moments = cv2.moments(biggest)
        if moments["m00"] == 0:
            return None
        return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])

    def poll(self):
        ok, frame = self._capture.read()
        if not ok:
            self.note = "камера не отдала кадр"
            self.funnel.lost(self.note)
            return None
        self.funnel.hit("кадров")

        frame = self._cv2.flip(frame, 1)
        self.last_frame = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        hsv = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2HSV)

        found = {}
        for role in ("shoulder", "elbow", "wrist"):
            point = self._find(hsv, self.ranges[role])
            if point is None:
                self.note = f"метка «{role}» не найдена"
                self.funnel.lost(self.note)
                self.last_points = None
                return None
            found[role] = point
        self.funnel.hit("поза найдена")
        self.funnel.hit("точки видны")

        self.note = ""
        self.last_points = (found["shoulder"], found["elbow"], found["wrist"])
        return self.last_points

    def close(self):
        self._capture.release()


def open_camera(cv2, index: int):
    capture = cv2.VideoCapture(index)
    if not capture.isOpened():
        raise SystemExit(
            f"камера {index} не открылась.\n\n"
            "На macOS доступ выдаётся приложению, из которого запущен python,\n"
            "и подхватывается только НОВЫМ процессом:\n"
            "  1. разрешить в диалоге или в Системные настройки ->\n"
            "     Конфиденциальность и безопасность -> Камера;\n"
            "  2. перезапустить Терминал целиком;\n"
            "  3. запустить скрипт снова."
        )
    return capture


# --- отрисовка ------------------------------------------------------------
#
# Смысл картинки не в красоте. Рядом рисуются две вещи: что камера увидела в
# вашей руке и что из этого получилось у робота. Пока они не совпадают —
# приводы включать нельзя. Одного взгляда достаточно, чтобы поймать
# перевёрнутый знак, спутанные суставы или мусорное распознавание, а по
# цифрам это ловится долго.

COLOR_BONE = (74, 158, 255)
COLOR_JOINT = (255, 255, 255)
COLOR_ARC = (61, 220, 132)


def draw_arm(cv2, rgb, points, angles):
    """Нанести на кадр звенья, суставы и дуги углов."""
    frame = rgb.copy()
    if points is None:
        return frame

    shoulder, elbow, wrist = (tuple(map(int, p)) for p in points)
    cv2.line(frame, shoulder, elbow, COLOR_BONE, 4)
    cv2.line(frame, elbow, wrist, COLOR_BONE, 4)
    for point, radius in ((shoulder, 9), (elbow, 9), (wrist, 7)):
        cv2.circle(frame, point, radius, COLOR_JOINT, -1)
        cv2.circle(frame, point, radius, (0, 0, 0), 2)

    if angles is not None:
        _arc(cv2, frame, shoulder, 0.0, angles.q1_deg, f"q1 {angles.q1_deg:+.0f}")
        _arc(cv2, frame, elbow, angles.q1_deg, angles.q2_deg,
             f"q2 {angles.q2_deg:+.0f}")
    return frame


def _arc(cv2, frame, centre, base_deg, sweep_deg, label):
    """Дуга угла от base_deg на sweep_deg. Y на экране вниз, отсюда минусы."""
    radius = 46
    start = -base_deg
    end = -(base_deg + sweep_deg)
    cv2.ellipse(frame, centre, (radius, radius), 0.0,
                min(start, end), max(start, end), COLOR_ARC, 3)
    cv2.putText(frame, label, (centre[0] + 14, centre[1] - radius - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, COLOR_ARC, 2)


def draw_robot(canvas, width, height, q1_deg, q2_deg, tracked=None):
    """Схема робота сверху по КОМАНДНЫМ углам осей.

    Рядом бледным контуром — поза руки человека. Расхождение между ними и
    есть отставание робота: видно сразу, без чтения таблицы.
    """
    canvas.delete("all")
    origin_x, origin_y = width * 0.5, height * 0.72
    l1, l2 = height * 0.30, height * 0.24

    for radius in (l1 + l2, l1):
        canvas.create_oval(origin_x - radius, origin_y - radius,
                           origin_x + radius, origin_y + radius,
                           outline="#2c2c2c")

    def chain(q1, q2, colour, width_px, tag):
        a1 = math.radians(q1)
        elbow = (origin_x + l1 * math.cos(a1), origin_y - l1 * math.sin(a1))
        a2 = math.radians(q1 + q2)
        tip = (elbow[0] + l2 * math.cos(a2), elbow[1] - l2 * math.sin(a2))
        canvas.create_line(origin_x, origin_y, *elbow, fill=colour,
                           width=width_px, capstyle="round", tags=tag)
        canvas.create_line(*elbow, *tip, fill=colour, width=width_px,
                           capstyle="round", tags=tag)
        return elbow, tip

    if tracked is not None:
        chain(tracked[0], tracked[1], "#4a5a6a", 4, "human")

    elbow, tip = chain(q1_deg, q2_deg, "#3ddc84", 7, "robot")
    for point in ((origin_x, origin_y), elbow, tip):
        canvas.create_oval(point[0] - 5, point[1] - 5, point[0] + 5,
                           point[1] + 5, fill="#ffffff", outline="")
    canvas.create_text(width * 0.5, height - 14, fill="#777",
                       text="зелёное — робот, серое — ваша рука")


class VisionWorker:
    """Камера, распознавание и фильтр — в отдельном потоке.

    Раньше всё это крутилось прямо в колбэке Tk. Даже когда каждый шаг быстрый,
    их сумма — ожидание кадра от камеры, инференс, фильтр — складывается в
    десятки миллисекунд, и на это время окно перестаёт отвечать: не
    перерисовывается, не реагирует на кнопки. Снаружи это выглядит как
    зависание, хотя программа работает.

    Здесь поток гонит распознавание в своём темпе и складывает результат в
    ``snapshot``. Окно только читает готовое и рисует, поэтому остаётся
    отзывчивым при любой скорости распознавания. Кнопка СТОП обязана
    нажиматься всегда.
    """

    def __init__(self, source, arm_filter, on_target=None):
        self.source = source
        self.filter = arm_filter
        self.on_target = on_target

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self.frame = None
        self.points = None
        self.angles = None
        self.smoothed = None
        self.frame_id = 0
        self.poll_ms = 0.0
        self.fps = 0.0
        self.errors = 0
        self.last_error = ""

    def snapshot(self):
        with self._lock:
            return (self.frame, self.points, self.angles, self.smoothed,
                    self.frame_id)

    def _worker(self):
        recent = deque(maxlen=20)
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._one_frame()
            except Exception as exc:
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                log.exception("сбой при обработке кадра")
                time.sleep(0.05)      # не крутить сбой на полной скорости

            elapsed = time.monotonic() - started
            self.poll_ms = elapsed * 1000
            recent.append(started)
            if len(recent) > 1:
                span = recent[-1] - recent[0]
                self.fps = (len(recent) - 1) / span if span > 0 else 0.0

    def _one_frame(self):
        points = self.source.poll()
        angles = None
        if points is not None:
            try:
                angles = angles_from_points(*points)
                self.source.funnel.hit("углы посчитаны")
            except ValueError as exc:
                self.source.funnel.lost(f"геометрия: {exc}")

        smoothed = None
        if angles is not None:
            smoothed = self.filter.update(angles)
            if smoothed is None:
                self.source.funnel.lost("фильтр отбросил кадр")
            else:
                self.source.funnel.hit("фильтр принял")

        with self._lock:
            self.frame = self.source.last_frame
            self.points = self.source.last_points
            self.angles = angles
            if smoothed is not None:
                self.smoothed = smoothed
            self.frame_id += 1

        if smoothed is not None and self.on_target is not None:
            self.on_target(smoothed)

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


def preflight_line(transport: SerialConfig) -> None:
    """Убедиться, что линия молчит, пока молчим мы.

    Без этой проверки шумящая шина даёт не внятную ошибку, а зависание:
    поток мусора не даёт чтению завершиться по таймауту.
    """
    from scara_drive.discovery import measure_line_noise

    noise = measure_line_noise(transport.port, transport.baudrate,
                               transport.parity, seconds=0.5)
    if noise == 0:
        log.info("линия чистая")
        return

    raise SystemExit(
        f"\nЛИНИЯ ШУМИТ: с молчащей шины пришло {noise} байт за 0.5 с.\n\n"
        "Работать так нельзя — обмен будет зависать, а приводы поедут по\n"
        "мусорным показаниям положения.\n\n"
        "Две обычные причины:\n"
        "  1. на шине второй ведущий — закройте другие программы с этим\n"
        "     портом, в том числе C30 Servo GUI;\n"
        "  2. линия без смещения и терминаторов — подробности:\n"
        "     python tools/bus_health.py\n\n"
        "Запустить вопреки проверке: --skip-line-check\n"
    )


def build_followers(args, axes):
    config = FollowConfig(
        counts_per_rev=args.cpr, invert_encoder=not args.no_invert,
        kp=args.kp, max_rpm=args.max_rpm, tolerance_deg=args.tolerance,
        period_s=args.period,
    )
    warning = config.check_tolerance()
    if warning:
        log.warning(warning)

    if args.simulate:
        return ({a.name: AngleFollower(SimulatedAxis(args.cpr, not args.no_invert),
                                       config) for a in axes}, None)

    if args.host:
        transport = TcpConfig(host=args.host, port=args.tcp_port)
    else:
        port = args.port or guess_port()
        if port is None:
            raise SystemExit("не нашёл переходник — укажите --port")
        transport = SerialConfig(port=port, baudrate=args.baud, parity=args.parity)

    if isinstance(transport, SerialConfig) and not args.skip_line_check:
        preflight_line(transport)

    bus = DriveBus(transport)
    bus.connect()
    log.info("шина: %s", bus.where)
    return ({a.name: AngleFollower(bus.axis(a.device_id), config) for a in axes},
            bus)


def run_gui(loop, source, axes, arm_filter, args):
    import tkinter as tk
    from tkinter import ttk
    from PIL import Image, ImageTk

    keep = {}
    paused = {"value": True}          # стартуем на паузе: сначала проверьте картинку
    zero = {"q1": 0.0, "q2": 0.0}     # поза, принятая за ноль

    root = tk.Tk()
    root.title("SCARA — повтор позы руки")
    root.minsize(1180, 760)

    ttk.Label(root, font=("SF Pro Text", 13, "bold"),
              text="камера сверху: плечо -> J1, локоть -> J2").pack(pady=(12, 2))
    ttk.Label(root, font=("SF Pro Text", 11), foreground="#888",
              text="сначала убедитесь, что скелет лёг на руку правильно, "
                   "потом «ноль здесь» и снимайте паузу").pack(pady=(0, 8))

    middle = ttk.Frame(root)
    middle.pack(fill="both", expand=True, padx=14)

    video = tk.Label(middle, background="#111", foreground="#777",
                     text="ожидание кадра...", font=("SF Pro Text", 12))
    video.grid(row=0, column=0, sticky="nsew")
    robot = tk.Canvas(middle, width=380, height=430, background="#151515",
                      highlightthickness=0)
    robot.grid(row=0, column=1, padx=(14, 0), sticky="ns")
    middle.grid_columnconfigure(0, weight=1)

    table = ttk.Frame(root)
    table.pack(fill="x", padx=16, pady=10)
    headers = ("ось", "id", "сустав", "цель", "факт", "ошибка", "об/мин", "состояние")
    for column, title in enumerate(headers):
        ttk.Label(table, text=title, font=("SF Pro Text", 11, "bold")).grid(
            row=0, column=column, padx=8, sticky="w")
    table.grid_columnconfigure(7, weight=1)

    rows = {}
    for index, axis in enumerate(axes, start=1):
        labels = []
        for column in range(len(headers)):
            label = ttk.Label(table, text="—", font=("SF Pro Text", 12))
            label.grid(row=index, column=column, padx=8, sticky="w")
            labels.append(label)
        labels[0].config(text=axis.name)
        labels[1].config(text=str(axis.device_id))
        labels[2].config(text="плечо" if axis.joint == "q1" else "локоть")
        rows[axis.name] = labels

    status = ttk.Label(root, text="", font=("SF Pro Text", 11))
    status.pack(pady=(0, 4))
    detector = ttk.Label(root, text="", font=("SF Pro Text", 11), foreground="#888")
    detector.pack(pady=(0, 8))

    buttons = ttk.Frame(root)
    buttons.pack(pady=(0, 14))

    def toggle_pause():
        if not zeroed["value"]:
            status.config(text="сначала «ноль здесь»: без калибровки цель "
                               "считается от произвольного угла",
                          foreground="#ffb648")
            return
        paused["value"] = not paused["value"]
        pause_button.config(text="ПРОДОЛЖИТЬ" if paused["value"] else "ПАУЗА")

    def emergency_stop():
        paused["value"] = True
        loop.stop()
        pause_button.config(text="ПРОДОЛЖИТЬ")
        status.config(text="ОСТАНОВЛЕНО кнопкой. Перезапустите приложение.",
                      foreground="#e05252")

    def set_zero():
        current = arm_filter.last
        if current is None:
            status.config(text="руку не вижу — нечего принимать за ноль",
                          foreground="#ffb648")
            return
        zero["q1"], zero["q2"] = current
        loop.set_zero_here()
        zeroed["value"] = True
        status.config(text=f"ноль принят: плечо {zero['q1']:.0f}°, "
                           f"локоть {zero['q2']:.0f}°. Можно снимать паузу.",
                      foreground="")

    pause_button = ttk.Button(buttons, text="ПРОДОЛЖИТЬ", command=toggle_pause)
    pause_button.grid(row=0, column=0, padx=6)
    ttk.Button(buttons, text="СТОП", command=emergency_stop).grid(
        row=0, column=1, padx=6)
    ttk.Button(buttons, text="ноль здесь", command=set_zero).grid(
        row=0, column=2, padx=6)

    state = {"errors": 0, "last_error": "", "shown_frame": -1}
    zeroed = {"value": False}

    def on_target(smoothed):
        """Вызывается из потока распознавания."""
        if paused["value"] or not zeroed["value"]:
            return
        relative = (wrap180(smoothed[0] - zero["q1"]),
                    wrap180(smoothed[1] - zero["q2"]))
        loop.set_targets({
            axis.name: axis.target_deg(
                relative[0] if axis.joint == "q1" else relative[1])
            for axis in axes
        })

    worker = VisionWorker(source, arm_filter, on_target)

    def draw():
        try:
            draw_once()
        except Exception as exc:
            state["errors"] += 1
            state["last_error"] = f"отрисовка: {type(exc).__name__}: {exc}"
            log.exception("сбой отрисовки")
        finally:
            root.after(50, draw)

    def draw_once():
        import cv2
        frame, points, angles, smoothed, frame_id = worker.snapshot()

        # Пересобираем картинку только на новом кадре: превращение массива в
        # изображение Tk стоит дорого, а распознавание идёт своим темпом.
        if frame is not None and frame_id != state["shown_frame"]:
            state["shown_frame"] = frame_id
            painted = draw_arm(cv2, frame, points, angles)
            image = Image.fromarray(painted)
            image.thumbnail((720, 430))
            photo = ImageTk.PhotoImage(image)
            keep["frame"] = photo
            video.configure(image=photo, text="")

        commanded = []
        for axis in axes:
            telemetry = loop.followers[axis.name].telemetry
            labels = rows[axis.name]
            labels[3].config(text=f"{telemetry.target_deg:7.1f}°")
            labels[4].config(text=f"{telemetry.actual_deg:7.1f}°")
            labels[5].config(text=f"{telemetry.error_deg:+7.1f}°")
            labels[6].config(text=f"{telemetry.command_rpm:+3d}")
            labels[7].config(text="удержание" if telemetry.holding else "едет")
            commanded.append(wrap180(telemetry.actual_deg))

        human = None
        if smoothed is not None and zeroed["value"]:
            human = (wrap180(smoothed[0] - zero["q1"]),
                     wrap180(smoothed[1] - zero["q2"]))
        draw_robot(robot, 380, 430, commanded[0], commanded[1], human)

        if loop.error_text:
            status.config(text=f"ОСТАНОВЛЕНО: {loop.error_text}",
                          foreground="#e05252")
        elif loop.warning_text:
            status.config(text=loop.warning_text, foreground="#ffb648")
        elif not zeroed["value"]:
            status.config(text="сначала нажмите «ноль здесь» — без этого "
                               "приводы не поедут", foreground="#ffb648")
        elif paused["value"]:
            status.config(text="ПАУЗА — приводы не двигаются", foreground="#ffb648")
        else:
            status.config(text=f"ведём   обход шины {loop.sweep_ms:.0f} мс   "
                               f"всего {loop.sweeps}", foreground="")

        note = source.funnel.last_loss or source.note
        errors = state["errors"] + worker.errors
        tail = ""
        if errors:
            tail = f"   СБОЕВ {errors}: {worker.last_error or state['last_error']}"
        detector.config(
            text=f"распознавание {worker.fps:.0f} к/с ({worker.poll_ms:.0f} мс)   "
                 f"{source.funnel.report()}   |   {note}{tail}",
            foreground="#e05252" if errors else "#888")

    def shutdown():
        worker.stop()
        loop.stop()
        source.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", shutdown)
    loop.start()
    worker.start()
    draw()
    try:
        root.mainloop()
    finally:
        worker.stop()
        loop.stop()
        source.close()
    return 0


def run_diag(source, arm_filter, seconds: float) -> int:
    """Померить, на какой стадии теряются кадры. Без окна и без приводов."""
    print(f"замер {seconds:g} с — держите руку в кадре и подвигайте ей\n")
    start = time.monotonic()
    last_print = 0.0

    while time.monotonic() - start < seconds:
        points = source.poll()
        if points is not None:
            try:
                angles = angles_from_points(*points)
                source.funnel.hit("углы посчитаны")
                if arm_filter.update(angles) is not None:
                    source.funnel.hit("фильтр принял")
                else:
                    source.funnel.lost("фильтр отбросил кадр")
            except ValueError as exc:
                source.funnel.lost(f"геометрия: {exc}")

        elapsed = time.monotonic() - start
        if elapsed - last_print >= 1.0:
            last_print = elapsed
            print(f"  {elapsed:4.0f} с   {source.funnel.report()}")

    counts = source.funnel.counts
    frames = counts["кадров"]
    print("\n" + "=" * 62)
    if not frames:
        print("камера не отдала ни одного кадра.")
        return 1

    print(f"{'стадия':<18} {'кадров':>8} {'доля':>7}   потеряно на шаге")
    print("-" * 62)
    previous = frames
    worst = (0, "")
    for stage in Funnel.STAGES:
        value = counts[stage]
        dropped = previous - value if stage != "кадров" else 0
        if dropped > worst[0]:
            worst = (dropped, stage)
        print(f"{stage:<18} {value:>8} {value / frames * 100:>6.0f}%   "
              f"{dropped if stage != 'кадров' else '':>6}")
        previous = value

    passed = counts["фильтр принял"]
    print("-" * 62)
    print(f"\nдо привода дошло {passed} из {frames} кадров "
          f"({passed / frames * 100:.0f}%), это {passed / seconds:.1f} обновлений в секунду")

    if passed / frames > 0.7:
        print("\nТракт распознавания в порядке.")
    elif worst[1] == "поза найдена":
        print("\nБольше всего теряется на поиске позы: модель не видит человека.")
        print("Вид сверху для неё нетипичен. Что пробовать:")
        print("  * попасть в кадр целиком — плечи и корпус, не только руку;")
        print("  * добавить света, убрать контровой свет за спиной;")
        print("  * --source markers — три цветные метки, ракурс им безразличен.")
    elif worst[1] == "точки видны":
        print("\nПоза находится, но конкретный сустав модель считает скрытым.")
        print("Строка «потеряно» выше называет какой. Что пробовать:")
        print("  * повернуть камеру так, чтобы сустав не перекрывался;")
        print("  * --side left, если управляете другой рукой;")
        print("  * ослабить порог: --min-visibility 0.2")
    elif worst[1] == "фильтр принял":
        print("\nТочки находятся, но фильтр считает движение неправдоподобным.")
        print("Обычно это дёрганое распознавание. Что пробовать:")
        print("  * --max-len-change 0.4 и --max-jump 70 — ослабить отсев;")
        print("  * --source markers — распознавание стабильнее в принципе.")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=["pose", "markers"], default="pose")
    p.add_argument("--side", choices=["left", "right"], default="right",
                   help="какой рукой управляете (для источника pose)")
    p.add_argument("--simulate", action="store_true")
    p.add_argument("--confirm", action="store_true",
                   help="подтвердить, что оси разгружены")

    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--parity", default="E", choices=["N", "E", "O"])
    p.add_argument("--host")
    p.add_argument("--tcp-port", type=int, default=502)

    p.add_argument("--cpr", type=int, default=131072)
    p.add_argument("--no-invert", action="store_true")
    p.add_argument("--max-rpm", type=int, default=15)
    p.add_argument("--kp", type=float, default=0.225)
    p.add_argument("--tolerance", type=float, default=2.5)
    p.add_argument("--period", type=float, default=0.12)
    p.add_argument("--sweep", type=float, default=0.12)
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--smoothing", type=float, default=0.45)
    p.add_argument("--diag", action="store_true",
                   help="замерить, где теряются кадры, и выйти")
    p.add_argument("--diag-seconds", type=float, default=12.0)
    p.add_argument("--min-visibility", type=float, default=0.5,
                   help="порог видимости сустава, ниже — кадр отбрасывается")
    p.add_argument("--max-len-change", type=float, default=0.25,
                   help="допустимый скачок длины звена за кадр")
    p.add_argument("--max-jump", type=float, default=45.0,
                   help="допустимый скачок угла за кадр, градусы")
    p.add_argument("--skip-line-check", action="store_true",
                   help="не проверять шину на шум перед запуском")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(message)s")

    if not (args.simulate or args.confirm or args.diag):
        print(__doc__)
        print("Запустите с --confirm, когда оси будут разгружены, "
              "с --simulate для проверки без железа "
              "или с --diag, чтобы померить распознавание.")
        return 1

    if args.diag:
        source = (PoseArmSource(args.camera, args.side) if args.source == "pose"
                  else MarkerArmSource(args.camera))
        source.MIN_VISIBILITY = args.min_visibility
        arm_filter = ArmFilter(smoothing=args.smoothing,
                               max_len_change=args.max_len_change,
                               max_jump_deg=args.max_jump)
        try:
            return run_diag(source, arm_filter, args.diag_seconds)
        finally:
            source.close()

    axes = load_axes()
    print("оси:")
    for axis in axes:
        joint = "плечо" if axis.joint == "q1" else "локоть"
        print(f"  id={axis.device_id}  {axis.name:<10} <- {joint}, "
              f"ход ±{axis.span_deg / 2:.0f}°, усиление {axis.gain:g}")
    print()

    followers, bus = build_followers(args, axes)
    loop = MultiAxisFollower(followers, period_s=args.sweep)
    loop.set_zero_here()

    source = (PoseArmSource(args.camera, args.side) if args.source == "pose"
              else MarkerArmSource(args.camera))
    source.MIN_VISIBILITY = args.min_visibility
    arm_filter = ArmFilter(smoothing=args.smoothing,
                           max_len_change=args.max_len_change,
                           max_jump_deg=args.max_jump)

    try:
        return run_gui(loop, source, axes, arm_filter, args)
    finally:
        if bus is not None:
            try:
                bus.stop_all()
            finally:
                bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
