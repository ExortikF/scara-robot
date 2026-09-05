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
import os
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, fields
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import DriveBus, SerialConfig, TcpConfig
from scara_drive.armtrack import ArmFilter, angles_from_points, wrap180
from scara_drive.discovery import guess_port
from scara_drive.simulation import SimulatedAxis
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
    """Ось робота: связь с суставом человека и механика привода.

    ═══ ПЕРЕДАТОЧНОЕ ЧИСЛО ═══

    ``gear_ratio`` — сколько оборотов делает ВАЛ МОТОРА на один оборот
    СУСТАВА. Редуктор 1:10 — это ``gear_ratio = 10``: мотор крутится
    вдесятеро быстрее сустава, а момента даёт вдесятеро больше.

    Если передача повышающая (большой шкив ведёт малый, сустав крутится
    быстрее мотора), число меньше единицы: ремень «полтора наоборот» —
    это ``gear_ratio = 1 / 1.5 = 0.667``.

    Отсюда пересчёт, который иначе легко перепутать:

        градусы вала = градусы сустава * gear_ratio

    ``reverse`` — переворачивает ли передача направление. Зубчатая пара
    переворачивает, ремень между шкивами — нет.

    ═══ ПРЕДЕЛЫ ═══

    ``min_deg`` и ``max_deg`` задаются в градусах СУСТАВА и отсчитываются
    от середины, выставленной калибровкой. По умолчанию ±90°.
    """

    name: str
    device_id: int
    joint: str                    # "q1" — плечо, "q2" — локоть
    gear_ratio: float = 1.0       # оборотов вала на оборот сустава
    reverse: bool = False         # передача переворачивает направление
    min_deg: float = -100.0       # предел сустава от середины
    max_deg: float = 100.0
    gain: float = 1.0             # во сколько раз повторять движение руки

    #: Механический упор в абсолютной шкале привода 0..360° — той самой,
    #: что показывает штатная программа. Привязан к железу и не съезжает
    #: при смене середины. None означает «упора нет».
    abs_zero_counts: int | None = None
    abs_min_deg: float = 0.0
    abs_max_deg: float = 360.0
    max_joint_deg_s: float = 60.0 # потолок скорости сустава

    def __post_init__(self) -> None:
        if self.gear_ratio <= 0:
            raise ValueError(f"{self.name}: gear_ratio должен быть больше нуля")
        if self.min_deg >= self.max_deg:
            raise ValueError(f"{self.name}: min_deg должен быть меньше max_deg")

    # --- сустав <-> вал ---------------------------------------------------

    def joint_to_motor(self, joint_deg: float) -> float:
        value = joint_deg * self.gear_ratio
        return -value if self.reverse else value

    def motor_to_joint(self, motor_deg: float) -> float:
        value = motor_deg / self.gear_ratio
        return -value if self.reverse else value

    # --- цель -------------------------------------------------------------

    def joint_target(self, human_deg: float) -> float:
        """Угол сустава человека -> угол сустава робота, с учётом пределов."""
        return max(self.min_deg, min(human_deg * self.gain, self.max_deg))

    def motor_target(self, human_deg: float) -> float:
        """То же, но сразу в градусах вала — это и уходит в контур."""
        return self.joint_to_motor(self.joint_target(human_deg))

    def is_clamped(self, human_deg: float) -> bool:
        value = human_deg * self.gain
        return not (self.min_deg < value < self.max_deg)

    @property
    def max_motor_rpm(self) -> int:
        """Потолок оборотов вала под заданную скорость сустава."""
        return max(1, round(self.max_joint_deg_s * self.gear_ratio / 6.0))

    def limits_text(self) -> str:
        return f"{self.min_deg:+.0f}..{self.max_deg:+.0f}°"

    def gear_text(self) -> str:
        if self.gear_ratio >= 1:
            base = f"редуктор 1:{self.gear_ratio:g}"
        else:
            base = f"повышающая {1 / self.gear_ratio:g}:1"
        return base + (", реверс" if self.reverse else "")


DEFAULT_AXES = [
    # J1: редуктор с передаточным 10.
    #     ±90° сустава — это ±900° вала, два с половиной оборота в каждую
    #     сторону. Ради этого контур и переведён в многооборотный режим.
    ArmAxis("J1 плечо", 1, "q1", gear_ratio=10.0),

    # J2: ремень ГРМ, большой шкив на моторе ведёт малый на суставе.
    #     Передача ПОВЫШАЮЩАЯ: сустав крутится в 1.5 раза быстрее вала,
    #     значит gear_ratio = 1/1.5. ±90° сустава — всего ±60° вала.
    #     Обратная сторона: момента на суставе в 1.5 раза МЕНЬШЕ, чем
    #     отдаёт мотор.
    ArmAxis("J2 локоть", 2, "q2", gear_ratio=1.0 / 1.5),
]


def load_axes() -> list[ArmAxis]:
    """Прочитать описание осей, переварив файлы от прежних версий.

    Описание менялось по мере того, как собиралась машина: сначала был
    симметричный span_deg, потом пара пределов, теперь ещё и передачи.
    Ронять запуск из-за старого файла нельзя — приводим что есть к
    текущему виду и говорим, что поменяли.
    """
    if not AXES_FILE.is_file():
        save_axes(DEFAULT_AXES)
        log.info("создан %s — проверьте передачи и пределы", AXES_FILE.name)
        return DEFAULT_AXES

    known = {f.name for f in fields(ArmAxis)}
    defaults = {a.name: a for a in DEFAULT_AXES}
    axes, changed = [], False

    for item in json.loads(AXES_FILE.read_text(encoding="utf-8")):
        item = dict(item)

        # Симметричный ход из самой первой версии.
        if "span_deg" in item:
            half = item.pop("span_deg") / 2
            item.setdefault("min_deg", -half)
            item.setdefault("max_deg", half)
            changed = True

        # «Без предела» из версии до установки редукторов: с механикой на
        # месте безграничный ход опасен, возвращаем ±90°.
        for side, fallback in (("min_deg", -100.0), ("max_deg", 100.0)):
            if item.get(side) is None:
                item[side] = fallback
                changed = True

        dropped = set(item) - known
        for key in dropped:
            item.pop(key)
        if dropped:
            changed = True

        # Передачи в старых файлах не описаны — берём из умолчаний по имени.
        if "gear_ratio" not in item:
            sample = defaults.get(item.get("name"))
            if sample is not None:
                item["gear_ratio"] = sample.gear_ratio
                item["reverse"] = sample.reverse
                changed = True

        axes.append(ArmAxis(**item))

    if changed:
        save_axes(axes)
        log.warning("%s обновлён под новую механику — ПРОВЕРЬТЕ передачи "
                    "и пределы", AXES_FILE.name)
    return axes


def save_axes(axes: list[ArmAxis]) -> None:
    AXES_FILE.write_text(
        json.dumps([asdict(a) for a in axes], ensure_ascii=False, indent=2),
        encoding="utf-8")


class PoseArmSource:
    """Плечо, локоть и запястье с mediapipe Pose."""

    name = "pose"
    MODEL_NAME = "pose_landmarker_full.task"
    MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
                 "pose_landmarker_full/float16/1/pose_landmarker_full.task")

    #: Ниже этого mediapipe сам не уверен в точке — такой кадр не берём.
    MIN_VISIBILITY = 0.5

    def __init__(self, camera_index: int = 0, side: str = "right",
                 model_path: Path | None = None, realsense: bool = False,
                 allow_reset: bool = False):
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
        self._capture = (RealSenseFrames(allow_reset=allow_reset) if realsense
                         else open_camera(cv2, camera_index))
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

    def __init__(self, camera_index: int = 0, realsense: bool = False,
                 allow_reset: bool = False):
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

        self._capture = (RealSenseFrames(allow_reset=allow_reset) if realsense
                         else open_camera(cv2, camera_index))
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


def camera_names() -> list[str]:
    """Имена камер от системы.

    OpenCV на macOS имён не отдаёт, а system_profiler отдаёт. Порядок в
    его выводе обычно совпадает с индексами AVFoundation, но гарантии нет,
    поэтому имена показываем как подсказку, а не как точное соответствие.
    """
    import subprocess

    try:
        out = subprocess.run(
            ["system_profiler", "SPCameraDataType"],
            capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return []

    names = []
    for line in out.splitlines():
        stripped = line.strip()
        # Имя устройства — строка с двоеточием на втором уровне отступа.
        if (stripped.endswith(":") and not stripped.startswith("Camera")
                and ":" not in stripped[:-1]
                and len(line) - len(line.lstrip()) == 4):
            names.append(stripped[:-1])
    return names


def list_cameras(max_index: int = 6, snapshot_dir: Path | None = None):
    """Найти доступные камеры перебором индексов.

    OpenCV на macOS не отдаёт имена устройств, только порядковые номера.
    Разрешение тоже различает не всегда: встроенная камера ноутбука и
    цветной поток RealSense обе дают 1920x1080. Поэтому с каждой камеры
    снимается кадр — по картинке видно сразу.

    RealSense подключается как обычная USB-камера, цветной поток берётся
    без всякого SDK. Карта глубины при этом недоступна, но она и не нужна:
    углы считаются по плоскому кадру.
    """
    import cv2

    found, misses = [], 0
    for index in range(max_index):
        capture = cv2.VideoCapture(index)
        if not capture.isOpened():
            capture.release()
            misses += 1
            # Пустых индексов подряд достаточно, чтобы остановиться: иначе
            # OpenCV засыпает терминал руганью про несуществующие камеры.
            if misses >= 2 and found:
                break
            continue

        misses = 0
        ok, frame = capture.read()
        if ok and frame is not None:
            height, width = frame.shape[:2]
            saved = None
            if snapshot_dir is not None:
                snapshot_dir.mkdir(parents=True, exist_ok=True)
                saved = snapshot_dir / f"camera{index}.jpg"
                cv2.imwrite(str(saved), frame)
            found.append((index, width, height, saved))
        capture.release()
    return found


class RealSenseFrames:
    """Цветной поток RealSense через SDK. Глубина не запрашивается вовсе.

    Открывать RealSense как обычную веб-камеру технически можно, но
    картинка выходит засвеченная и с цветовым перекосом: система отдаёт
    поток без нормальных настроек. Через pyrealsense2 камера отдаёт то,
    на что рассчитана.

    Подход перенесён из рабочего проекта arm_realsense:

    * VDCAssistant сгоняется с камеры — пока системный демон держит
      RGB-интерфейс, librealsense получает «failed to set power state»;
    * нужны права root, иначе macOS не отдаёт устройство библиотеке;
    * список режимов спрашиваем у самой камеры: он зависит от модели,
      прошивки и сильнее всего от того, USB 2 сейчас или USB 3;
    * режим считается годным, только когда с него ПРИШЁЛ КАДР —
      pipeline.start() соглашается и на режимы, которым не хватает
      полосы, а падает уже на ожидании кадра;
    * удачный режим запоминается: перебор честный, но медленный, и со
      стороны выглядит зависшим скриптом.

    Отличие от того проекта одно: там нужна была глубина, здесь нет.
    Отказ от неё освобождает половину полосы USB, поэтому проходят
    режимы, которые вдвоём с глубиной не тянулись.
    """

    #: Запасной список, если опросить камеру не удалось.
    FALLBACK_MODES = [(640, 480, 30), (640, 480, 15), (848, 480, 30),
                      (1280, 720, 30), (424, 240, 30)]

    MODE_CACHE = Path(__file__).with_name("realsense_mode.json")

    def __init__(self, width: int = 0, height: int = 0, fps: int = 0,
                 allow_reset: bool = False):
        """
        ``allow_reset`` — разрешить hardware_reset при неудаче.

        По умолчанию выключен намеренно. Сброс отключает устройство от
        шины и ждёт, что оно поднимется само, а поднимается оно не всегда:
        на этой машине камера после сброса пропадала из системы совсем, и
        вернуть её можно было только кабелем. Лекарство вышло опаснее
        болезни, поэтому теперь только по явной просьбе.
        """
        import os

        self._allow_reset = allow_reset
        self._errors: list[str] = []

        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise SystemExit(
                "pyrealsense2 не установлен в этом окружении.\n\n"
                "Запускайте из окружения, где он собран:\n"
                "    sudo ./.venv-rs/bin/python tools/arm_control.py "
                "--confirm --realsense\n"
            ) from exc
        self._rs = rs

        if os.geteuid() != 0:
            raise SystemExit(
                "Для RealSense нужны права root: без них macOS не отдаёт\n"
                "камеру библиотеке.\n\nЗапускайте так:\n"
                "    sudo killall VDCAssistant 2>/dev/null; \\\n"
                "    sudo ./.venv-rs/bin/python tools/arm_control.py "
                "--confirm --realsense\n"
            )

        self._warn_about_leftovers()
        self._release_daemon()

        # Контекст ОБЯЗАН умереть до создания конвейера.
        #
        # Пока он жив, устройство числится за ним, а rs.pipeline() заводит
        # свой контекст и камеру уже не находит: start() отвечает «No
        # device connected» сразу после того, как query_devices() её
        # прекрасно видел. Поэтому перечисление вынесено в метод, из
        # которого наружу выходят одни числа.
        modes = self._enumerate(width, height, fps)
        log.info("вариантов режима: %d, первым пробую %dx%d@%d",
                 len(modes), *modes[0])
        self.pipeline = None
        cached = self._load_mode()

        # Если режим из кэша уже работал, а сейчас не идёт — дело не в
        # режиме, а в занятой камере. Перебирать остальные бессмысленно:
        # они провалятся по той же причине, потратив по полторы секунды
        # каждый. Сразу сбрасываем устройство и повторяем кэш.
        if cached and cached in modes:
            if self._try_modes([cached]):
                return
            log.warning("режим из прошлого запуска не пошёл — камера занята")

            # Восстанавливаемся по возрастанию тяжести. Сброс железа —
            # мера последняя: он занимает секунды и сам требует
            # исключительного доступа, которого может не быть.
            log.info("шаг 1: отпускаю интерфейс и пробую снова")
            self._release_daemon(1.5)
            if self._try_modes([cached]):
                log.info("помогло освобождение интерфейса")
                return

            if self._allow_reset:
                log.info("шаг 2: сброс устройства")
                if self._reset_device() and self._try_modes([cached]):
                    log.info("после сброса открылась на прежнем режиме")
                    return

            log.info("шаг 2: перебираю остальные режимы")
            modes = [m for m in modes if m != cached]

        if self._try_modes(modes):
            return

        log.warning("камера не отдала кадр ни на одном режиме, "
                    "сбрасываю устройство")
        if self._allow_reset and self._reset_device() and self._try_modes(modes):
            log.info("после сброса камера открылась")
            return

        raise SystemExit(
            "\nКАМЕРА НЕ ОТКРЫВАЕТСЯ. Ни один режим не отдал кадр:\n"
            + "\n".join(self._errors) +
            "\n\n«failed to set power state» означает, что RGB-интерфейс\n"
            "камеры держит система, и отпускать не хочет.\n\n"
            "Что помогает, по порядку:\n"
            "  1. вынуть и вставить кабель RealSense — самое надёжное;\n"
            "  2. закрыть приложения, которые могли брать камеру:\n"
            "     Photo Booth, FaceTime, Zoom, вкладку браузера с видео;\n"
            "  3. убедиться, что не остался прошлый запуск:\n"
            "     pkill -f arm_control.py\n"
            "  4. кабель и разъём должны быть USB 3.\n\n"
            "Проверить, видит ли камеру система:\n"
            "    system_profiler SPCameraDataType | grep -i realsense\n"
            "Если пусто — камера отвалилась от USB, поможет только кабель.\n")

    #: Сколько ждать первый кадр при подборе режима. Три секунды на
    #: вариант при десятке вариантов складываются в полминуты молчания, и
    #: со стороны это неотличимо от зависшей программы. Полутора секунд
    #: рабочему режиму хватает с запасом: камера отдаёт кадры сразу.
    PROBE_TIMEOUT_MS = 1500

    @staticmethod
    def _warn_about_leftovers() -> None:
        """Найти незавершённые прошлые запуски.

        Они держат камеру, и это выглядит как «failed to set power state»
        при живом и видимом устройстве — то есть как беда с драйвером,
        хотя на самом деле мы сами себе мешаем. Отличить одно от другого
        по сообщению librealsense невозможно, поэтому проверяем прямо.
        """
        import os
        import subprocess

        try:
            out = subprocess.run(["pgrep", "-f", "arm_control.py"],
                                 capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            return

        mine = {os.getpid(), os.getppid()}
        others = [int(p) for p in out.stdout.split() if int(p) not in mine]
        if not others:
            return

        # Предупреждаем, но не запрещаем: определить «чужой» процесс
        # надёжно нельзя, а запрет на догадке останавливал бы работу на
        # ровном месте. Решение остаётся за человеком.
        log.warning("похоже, запущено ещё %d процессов arm_control (%s). "
                    "Если камера не откроется — закройте их: "
                    "sudo pkill -9 -f arm_control.py",
                    len(others), ", ".join(map(str, others)))

    @staticmethod
    def _release_daemon(settle_s: float = 0.6) -> None:
        """Согнать VDCAssistant с камеры.

        Демон захватывает RGB-интерфейс, как только камерой интересуется
        что-нибудь через AVFoundation — включая нашу же неудавшуюся
        попытку открыть поток. Пока он держит интерфейс, librealsense
        отвечает «failed to set power state». Убивать безопасно: он
        поднимается сам при следующем обращении системы к камере.

        Поэтому гнать его надо не один раз при старте, а перед каждой
        попыткой достучаться до камеры.
        """
        import subprocess

        subprocess.run(["killall", "VDCAssistant"],
                       capture_output=True, check=False)
        time.sleep(settle_s)         # дать интерфейсу освободиться

    def _reset_device(self) -> bool:
        """Сбросить камеру и заново её найти.

        Контекст живёт только внутри попытки: если он переживёт её,
        устройство останется за ним и конвейер его не увидит.
        """
        import gc

        rs = self._rs
        for attempt in (1, 2):
            self._release_daemon()
            context = devices = None
            try:
                context = rs.context()
                devices = list(context.query_devices())
                if not devices:
                    log.warning("камеру не видно, попытка %d", attempt)
                    continue
                devices[0].hardware_reset()
            except Exception as exc:
                log.warning("сброс, попытка %d: %s", attempt, exc)
                continue
            finally:
                devices = context = None
                gc.collect()

            time.sleep(4.0)          # даёт камере переподключиться
            self._release_daemon()
            return True
        return False

    def _try_modes(self, modes) -> bool:
        """Пройти по режимам, вернуть True на первом, отдавшем кадр."""
        rs = self._rs
        self._errors = []
        for number, (w, h, f) in enumerate(modes, 1):
            log.info("  %d/%d: пробую %dx%d@%d", number, len(modes), w, h, f)
            try:
                # Ровно как в рабочем проекте arm_realsense: конвейер без
                # явного контекста. Привязка к своему контексту выглядела
                # логичнее, но в проверенном коде её нет, а всё, что здесь
                # отличается от него, — лишний повод для отказа.
                pipeline = rs.pipeline()
                cfg = rs.config()
                # Только цвет. Глубину не включаем — она тут не нужна и
                # съедала бы полосу, из-за которой режим может не пойти.
                cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
                pipeline.start(cfg)
            except Exception as exc:
                self._errors.append(f"  {w}x{h}@{f}: не стартовал — {exc}")
                # Даже неудавшийся start() оставляет конвейер частично
                # захватившим устройство. Пока он не отпущен, ни повторная
                # попытка, ни сброс до камеры не достучатся.
                try:
                    pipeline.stop()
                except Exception:
                    pass
                del pipeline
                continue
            try:
                pipeline.wait_for_frames(self.PROBE_TIMEOUT_MS)
            except Exception as exc:
                self._errors.append(f"  {w}x{h}@{f}: кадров нет — {exc}")
                try:
                    pipeline.stop()
                except Exception:
                    pass
                continue
            self.pipeline = pipeline
            self._save_mode(w, h, f)
            log.info("RealSense: цветной поток %dx%d@%d", w, h, f)
            return True
        return False

    # --- выбор режима -----------------------------------------------------

    def _enumerate(self, width: int, height: int, fps: int
                   ) -> list[tuple[int, int, int]]:
        """Имя камеры и список режимов, ничего за собой не оставляя."""
        import gc

        rs = self._rs
        context = rs.context()
        devices = list(context.query_devices())
        if not devices:
            raise SystemExit(
                "RealSense не найден. Нужен USB 3 — и кабель, и разъём.")

        log.info("RealSense: %s", devices[0].get_info(rs.camera_info.name))
        if width and height and fps:
            modes = [(width, height, fps)]
        else:
            log.info("спрашиваю у камеры список режимов...")
            modes = self._modes_for(devices[0])

        # Ссылки рвём явно и добиваем сборщиком: иначе объекты доживут
        # как локальные до конца метода, и камера останется занята.
        devices.clear()
        del devices, context
        gc.collect()
        return modes

    def _modes_for(self, device) -> list[tuple[int, int, int]]:
        """Режимы цветного сенсора, от самых подходящих к прочим."""
        rs = self._rs
        found = set()
        for sensor in device.sensors:
            try:
                profiles = list(sensor.profiles)
            except Exception:
                continue      # сенсор занят — пропускаем, не роняя всё
            for profile in profiles:
                if "color" not in str(profile.stream_type()):
                    continue
                try:
                    video = profile.as_video_stream_profile()
                except Exception:
                    continue
                found.add((video.width(), video.height(), profile.fps()))

        modes = sorted(found | set(self.FALLBACK_MODES), key=self._rank)

        # Удачный прошлый режим — вперёд списка: перебор честный, но
        # медленный, каждый негодный режим съедает секунды.
        cached = self._load_mode()
        if cached:
            modes = [cached] + [m for m in modes if m != cached]
        return modes

    @staticmethod
    def _rank(mode) -> tuple:
        """Для слежения за рукой частота важнее числа пикселей."""
        w, h, f = mode
        return (0 if 15 <= f <= 30 else 1, abs(w * h - 640 * 480), -f)

    def _load_mode(self):
        try:
            data = json.loads(self.MODE_CACHE.read_text(encoding="utf-8"))
            return int(data["w"]), int(data["h"]), int(data["fps"])
        except Exception:
            return None

    def _save_mode(self, w: int, h: int, f: int) -> None:
        try:
            self.MODE_CACHE.write_text(
                json.dumps({"w": w, "h": h, "fps": f}), encoding="utf-8")
        except OSError:
            pass

    # --- как у cv2.VideoCapture -------------------------------------------

    def read(self):
        """Вернуть (ok, кадр BGR)."""
        import numpy as np

        try:
            frames = self.pipeline.wait_for_frames(2000)
        except Exception:
            return False, None
        colour = frames.get_color_frame()
        if not colour:
            return False, None
        return True, np.asanyarray(colour.get_data())

    def release(self):
        """Отпустить камеру.

        Вызывать обязательно: незакрытый конвейер оставляет устройство
        занятым, и следующий запуск получает «No device connected» при
        том, что камера в системе видна.
        """
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        except Exception:
            log.warning("конвейер RealSense не остановился штатно")
        finally:
            self.pipeline = None


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
COLOR_ARC = (61, 220, 132)

# Цвет суставов показывает состояние — его видно, не отрывая глаз от
# собственной руки, а строку состояния внизу окна в этот момент не
# прочитать. Кадр хранится в RGB, поэтому и тройки заданы как RGB.
JOINT_COLOURS = {
    "нет нуля": (74, 158, 255),     # синий: середина ещё не задана
    "стоп":     (224, 82, 82),      # красный: движение запрещено
    "работа":   (61, 220, 132),     # зелёный: идём за рукой
}


def draw_arm(cv2, rgb, points, angles, zero_q1=None, state="нет нуля"):
    """Нанести на кадр звенья, суставы и дуги углов.

    ``zero_q1`` — направление плеча в момент калибровки середины. Если оно
    известно, угол плеча показывается ОТНОСИТЕЛЬНО него: ноль в середине,
    рост в ту же сторону, куда пойдёт ось. Так число на экране совпадает
    с тем, что делает робот, и не возникает вопроса, от какой стороны
    отсчитывать. Без калибровки отсчёт идёт от горизонтали.
    """
    frame = rgb.copy()
    if points is None:
        return frame

    shoulder, elbow, wrist = (tuple(map(int, p)) for p in points)
    cv2.line(frame, shoulder, elbow, COLOR_BONE, 4)
    cv2.line(frame, elbow, wrist, COLOR_BONE, 4)
    colour = JOINT_COLOURS.get(state, JOINT_COLOURS["нет нуля"])
    for point, radius in ((shoulder, 9), (elbow, 9), (wrist, 7)):
        cv2.circle(frame, point, radius, colour, -1)
        cv2.circle(frame, point, radius, (0, 0, 0), 2)

    if angles is not None:
        if zero_q1 is None:
            _arc(cv2, frame, shoulder, 0.0, angles.q1_deg,
                 f"{angles.q1_deg:+.0f}")
        else:
            relative = wrap180(angles.q1_deg - zero_q1)
            _arc(cv2, frame, shoulder, zero_q1, relative, f"{relative:+.0f}")
            # Пунктиром — направление середины, чтобы было видно, от чего
            # отсчёт. Без этого «+15°» ни о чём не говорит.
            import math
            a = math.radians(-zero_q1)
            end = (int(shoulder[0] + 78 * math.cos(a)),
                   int(shoulder[1] + 78 * math.sin(a)))
            cv2.line(frame, shoulder, end, (120, 120, 120), 1)

        # Локоть рисуем ВНУТРИ сгиба: от направления «локоть -> плечо» к
        # «локоть -> запястье». Раньше дуга шла от продолжения плеча, то
        # есть по внешней стороне руки, и это выглядело неправильно —
        # человек ждёт увидеть угол между самими звеньями.
        #
        # Подпись тоже анатомическая: 180° у прямой руки, меньше по мере
        # сгибания. Управление по-прежнему считает q2 (отклонение от
        # прямой), но показывать удобнее привычную величину.
        back = angles.q1_deg + 180.0
        inner = 180.0 - angles.q2_deg
        _arc(cv2, frame, elbow, back, -inner, f"{abs(wrap180(inner)):.0f}",
             offset=(14, -54))
    return frame


def _arc(cv2, frame, centre, base_deg, sweep_deg, label, offset=(14, -54)):
    """Дуга от base_deg на sweep_deg. Y на экране вниз, отсюда минусы."""
    import math

    radius = 42
    start = -base_deg
    end = -(base_deg + sweep_deg)
    cv2.ellipse(frame, centre, (radius, radius), 0.0,
                min(start, end), max(start, end), COLOR_ARC, 3)

    # Подпись ставим на биссектрису дуги, а не в фиксированную точку:
    # иначе при некоторых позах она наезжает на звенья.
    middle = math.radians(-(base_deg + sweep_deg / 2.0))
    tx = int(centre[0] + (radius + 22) * math.cos(middle))
    ty = int(centre[1] + (radius + 22) * math.sin(middle))
    cv2.putText(frame, label, (tx - 14, ty + 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_ARC, 2)


class RobotView:
    """Схема робота сверху. Элементы создаются один раз и потом двигаются.

    Раньше на каждом кадре канва очищалась и рисовалась заново. Tk при этом
    успевает показать пустой холст между стиранием и отрисовкой, и окно
    заметно мигает. Здесь фигуры живут постоянно, а меняются только их
    координаты — мигания нет и рисуется дешевле.
    """

    def __init__(self, canvas, width: int, height: int):
        self.canvas = canvas
        self.width, self.height = width, height
        self.origin = (width * 0.5, height * 0.72)
        self.l1, self.l2 = height * 0.30, height * 0.24

        ox, oy = self.origin
        for radius in (self.l1 + self.l2, self.l1):
            canvas.create_oval(ox - radius, oy - radius, ox + radius,
                               oy + radius, outline="#2c2c2c")

        self.human = [canvas.create_line(0, 0, 0, 0, fill="#4a5a6a", width=4,
                                         capstyle="round") for _ in range(2)]
        self.robot = [canvas.create_line(0, 0, 0, 0, fill="#3ddc84", width=7,
                                         capstyle="round") for _ in range(2)]
        self.dots = [canvas.create_oval(0, 0, 0, 0, fill="#ffffff", outline="")
                     for _ in range(3)]
        canvas.create_text(width * 0.5, height - 14, fill="#777",
                           text="зелёное — робот, серое — ваша рука")

    def _chain(self, q1_deg: float, q2_deg: float):
        ox, oy = self.origin
        a1 = math.radians(q1_deg)
        elbow = (ox + self.l1 * math.cos(a1), oy - self.l1 * math.sin(a1))
        a2 = math.radians(q1_deg + q2_deg)
        tip = (elbow[0] + self.l2 * math.cos(a2), elbow[1] - self.l2 * math.sin(a2))
        return elbow, tip

    def update(self, q1_deg: float, q2_deg: float, tracked=None) -> None:
        ox, oy = self.origin

        if tracked is None:
            for item in self.human:
                self.canvas.coords(item, 0, 0, 0, 0)
        else:
            elbow, tip = self._chain(*tracked)
            self.canvas.coords(self.human[0], ox, oy, *elbow)
            self.canvas.coords(self.human[1], *elbow, *tip)

        elbow, tip = self._chain(q1_deg, q2_deg)
        self.canvas.coords(self.robot[0], ox, oy, *elbow)
        self.canvas.coords(self.robot[1], *elbow, *tip)
        for item, (x, y) in zip(self.dots, ((ox, oy), elbow, tip)):
            self.canvas.coords(item, x - 5, y - 5, x + 5, y + 5)


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


def check_speed_mode(bus, axes) -> None:
    """Убедиться, что приводы в скоростном режиме.

    В позиционном режиме (P-004 = 0) привод ждёт импульсы на входах
    PULS/SIGN и команды скорости по Modbus ИГНОРИРУЕТ. Внешне это
    выглядит хуже любой ошибки: программа работает, команды уходят,
    приводы отвечают — а вал стоит. Поэтому проверяем до запуска.
    """
    from scara_drive import registers as reg
    from scara_drive.driver import DriveError

    wrong = []
    for axis in axes:
        try:
            mode = bus.axis(axis.device_id).read_param(reg.P_CONTROL_MODE)
        except DriveError as exc:
            raise SystemExit(
                f"\nпривод {axis.device_id} ({axis.name}) не отвечает: {exc}\n\n"
                "Проверьте, кто на шине:\n"
                "    python examples/scan_bus.py <порт> --baud 19200 "
                "--parity E --ids 1-8\n") from exc
        if mode != reg.ControlMode.SPEED:
            wrong.append((axis, mode))

    if not wrong:
        return

    # Раньше здесь был отказ запускаться — и это была ошибка. Связь
    # «позиционный режим => JOG не работает» я не проверял: прочитал
    # P-004=0, сразу поменял на 1, оси поехали, и приписал заслугу смене
    # режима. JOG по руководству — пробный прогон, он может работать и
    # поверх режима. Пока не проверено на стенде, только предупреждаем.
    for axis, mode in wrong:
        log.warning("привод %d (%s): P-004 = %d, а не 1. Если ось не поедет, "
                    "попробуйте: python examples/mb.py write P-004 1 --id %d "
                    "--verify", axis.device_id, axis.name, mode, axis.device_id)
    return


def preflight_line(transport: SerialConfig) -> None:
    """Убедиться, что линия молчит, пока молчим мы.

    Без этой проверки шумящая шина даёт не внятную ошибку, а зависание:
    поток мусора не даёт чтению завершиться по таймауту.
    """
    from scara_drive.discovery import PortGoneError, measure_line_noise

    try:
        noise = measure_line_noise(transport.port, transport.baudrate,
                                   transport.parity, seconds=0.5)
    except PortGoneError as exc:
        raise SystemExit(f"\n{exc}\n") from exc
    if noise == 0:
        log.info("линия чистая")
        return

    # Раньше здесь был отказ запускаться. Это оказалось неверно: штатная
    # программа производителя на этой же линии работает, и наш контур тоже
    # — проверено ходом обеих осей. Шум мешает, но не запрещает, поэтому
    # предупреждаем и идём дальше.
    log.warning(
        "ЛИНИЯ ШУМИТ: с молчащей шины пришло %d байт за 0.5 с. Обмен будет "
        "срываться и подтормаживать. Причины и лечение: "
        "python tools/bus_health.py", noise)
    return


HOME_FILE = Path(__file__).with_name("arm_home.json")


def load_home() -> dict:
    """Исходное положение осей в ИМПУЛЬСАХ энкодера.

    Именно в импульсах, а не в градусах от программной середины: середину
    человек задаёт заново по многу раз за сеанс, и привязанное к ней
    исходное уезжало бы вместе с ней. Импульсы привязаны к валу.
    """
    if not HOME_FILE.is_file():
        return {}
    try:
        return {k: int(v) for k, v in
                json.loads(HOME_FILE.read_text(encoding="utf-8")).items()}
    except (OSError, ValueError) as exc:
        log.warning("не читается %s: %s", HOME_FILE.name, exc)
        return {}


def save_home(home: dict) -> None:
    try:
        HOME_FILE.write_text(json.dumps(home, ensure_ascii=False, indent=2),
                             encoding="utf-8")
    except OSError as exc:
        log.warning("не сохранить исходное: %s", exc)


def load_calibration() -> dict:
    """Сохранённые нули осей в импульсах энкодера.

    Энкодер абсолютный и многооборотный: он помнит положение вала и после
    выключения питания. Поэтому середину достаточно выставить один раз, а
    дальше она восстанавливается при каждом запуске — не нужно каждый раз
    руками ставить руку в среднее положение.
    """
    if not CALIBRATION_FILE.is_file():
        return {}
    try:
        return json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        log.warning("не читается %s: %s — калибровка сброшена",
                    CALIBRATION_FILE.name, exc)
        return {}


def save_calibration(data: dict) -> None:
    CALIBRATION_FILE.write_text(
        json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def build_followers(args, axes):
    """Собрать контуры по осям. У каждой оси свой предел оборотов."""
    if args.simulate:
        followers = {}
        for axis in axes:
            followers[axis.name] = AngleFollower(
                SimulatedAxis(args.cpr, not args.no_invert),
                axis_config(args, axis))
        return followers, None

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
    if not args.skip_mode_check:
        check_speed_mode(bus, axes)
    followers = {axis.name: AngleFollower(bus.axis(axis.device_id),
                                          axis_config(args, axis))
                 for axis in axes}
    return followers, bus


def axis_tolerance(args, axis: "ArmAxis") -> float:
    """Допуск оси в градусах вала.

    Задаётся в градусах СУСТАВА, чтобы одна цифра означала у всех осей
    одинаковую точность: из-за разных передач тот же допуск в градусах
    вала давал у локтя в пятнадцать раз более грубую зону, и он переставал
    замечать мелкие движения руки.

    Снизу ограничен путём, который ось проходит за цикл контура на
    минимальной скорости. Привод принимает целые об/мин и ниже одного не
    опускается, поэтому меньший допуск ось физически не выдержит — будет
    перескакивать зону и рыскать вокруг цели.
    """
    wanted = args.tolerance * abs(axis.gear_ratio)
    floor = 1.5 * 6.0 * args.period          # 1 об/мин = 6 град/с
    return max(wanted, floor)


def axis_config(args, axis: ArmAxis) -> FollowConfig:
    """Настройки контура под конкретную ось.

    Потолок оборотов свой у каждой оси: при редукторе 10 сустав на 30°/с
    требует от вала 300°/с, то есть 50 об/мин. Общий предел для всех осей
    означал бы, что либо тихоходная ось перекручивается, либо быстрая не
    успевает за рукой.
    """
    # Пределы сустава переводятся в градусы вала и уходят в контур как
    # жёсткая граница. Прижатие цели в ArmAxis — только первый слой; здесь
    # добавляется проверка фактического положения и направления движения.
    low = axis.joint_to_motor(axis.min_deg)
    high = axis.joint_to_motor(axis.max_deg)
    if low > high:                       # reverse переворачивает и пределы
        low, high = high, low

    config = FollowConfig(
        counts_per_rev=args.cpr,
        invert_encoder=not args.no_invert,
        kp=args.kp,
        max_rpm=args.max_rpm or axis.max_motor_rpm,
        # Допуск задаётся в градусах сустава и переводится в градусы вала
        # под передачу каждой оси. Иначе одна и та же цифра означает у
        # разных осей разную точность: при редукторе 10 и повышающей 1.5
        # разница выходит в пятнадцать раз, и ось с грубым допуском
        # перестаёт замечать мелкие движения руки.
        tolerance_deg=axis_tolerance(args, axis),
        # Шаг скорости считается из времени разгона, а не задаётся числом
        # об/мин: у осей разные потолки (150 и 10), и одна и та же добавка
        # означала бы у них совершенно разную плавность.
        max_rpm_step=max(0.5, (args.max_rpm or axis.max_motor_rpm)
                         * args.period / max(args.ramp, 0.05)),
        period_s=args.period,
        circular=False,          # ход ограничен, витки различать обязательно
        min_motor_deg=low,
        max_motor_deg=high,
        abs_zero_counts=axis.abs_zero_counts,
        abs_min_deg=axis.abs_min_deg,
        abs_max_deg=axis.abs_max_deg,
        limit_margin_deg=args.limit_margin,
        runaway_passes=args.runaway_passes,
    )
    warning = config.check_tolerance()
    if warning:
        log.warning("%s: %s", axis.name, warning)
    return config


def run_test_move(loop, axes, degrees: float, seconds: float) -> int:
    """Погонять оси настоящим контуром, но без камеры.

    Отделяет две вещи, которые снаружи выглядят одинаково: «не работает
    управление приводами» и «не распознаётся рука». Здесь цель ставится
    руками, поэтому если оси поехали — весь тракт до приводов исправен, и
    искать надо в распознавании.
    """
    import time

    print(f"\nпроверка хода: цель ±{degrees:g}° сустава, по {seconds:g} с\n")
    loop.start()
    loop.set_armed(True)
    try:
        for target in (degrees, -degrees, 0.0):
            print(f"  цель {target:+.0f}° сустава")
            loop.set_targets({a.name: a.motor_target(target) for a in axes})
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                time.sleep(0.4)
                parts = []
                for a in axes:
                    tel = loop.followers[a.name].telemetry
                    parts.append(f"{a.name.split()[0]} "
                                 f"{a.motor_to_joint(tel.actual_deg):+6.1f}° "
                                 f"{tel.command_rpm:+4d}об/м")
                print("     " + "   ".join(parts))
                if loop.error_text:
                    print(f"\n  АВАРИЯ: {loop.error_text}")
                    return 1
    finally:
        loop.stop()

    print("\nитог:")
    moved = False
    for a in axes:
        tel = loop.followers[a.name].telemetry
        joint = a.motor_to_joint(tel.actual_deg)
        print(f"  {a.name}: сустав {joint:+.1f}°, проходов контура {tel.passes}")
        if tel.passes > 5:
            moved = True

    if moved:
        print("\nконтур работает и команды до приводов доходят.")
        print("Если при этом рука не следует за вашей — дело в распознавании:")
        print("    python tools/arm_control.py --diag")
    else:
        print("\nконтур не сделал ни одного прохода — смотрите ошибки выше.")
    return 0


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
                   "потом «ноль здесь» и снимайте паузу").pack(pady=(0, 2))
    ttk.Label(root, font=("SF Pro Text", 11), foreground="#888",
              text="голосом: «старт» · «стоп» · «линия» · «положение»").pack(pady=(0, 8))

    middle = ttk.Frame(root)
    middle.pack(fill="both", expand=True, padx=14)

    video = tk.Label(middle, background="#111", foreground="#777",
                     text="ожидание кадра...", font=("SF Pro Text", 12))
    video.grid(row=0, column=0, sticky="nsew")
    robot = tk.Canvas(middle, width=380, height=430, background="#151515",
                      highlightthickness=0)
    robot.grid(row=0, column=1, padx=(14, 0), sticky="ns")
    robot_view = RobotView(robot, 380, 430)
    middle.grid_columnconfigure(0, weight=1)

    table = ttk.Frame(root)
    table.pack(fill="x", padx=16, pady=10)
    headers = ("ось", "id", "сустав", "цель", "факт", "ошибка", "об/мин",
               "угол привода", "состояние", "пределы")
    for column, title in enumerate(headers):
        ttk.Label(table, text=title, font=("SF Pro Text", 11, "bold")).grid(
            row=0, column=column, padx=8, sticky="w")
    table.grid_columnconfigure(9, weight=1)

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
        labels[9].config(text=axis.limits_text())
        rows[axis.name] = labels

    status = ttk.Label(root, text="", font=("SF Pro Text", 11))
    status.pack(pady=(0, 4))
    voice_label = ttk.Label(root, text="", font=("SF Pro Text", 12, "bold"))
    voice_label.pack(pady=(0, 4))
    detector = ttk.Label(root, text="", font=("SF Pro Text", 11),
                         foreground="#888")
    # Появляется по кнопке «диагностика», а не постоянно.

    buttons = ttk.Frame(root)
    buttons.pack(pady=(0, 14))

    def toggle_pause():
        if relaxed["value"]:
            status.config(text="сначала «ЗАЖАТЬ» — со снятым моментом "
                               "приводы не поедут", foreground="#ffb648")
            return
        if not zeroed["value"]:
            status.config(text="сначала выставьте середину: без неё цель "
                               "считается от произвольного угла",
                          foreground="#ffb648")
            return
        paused["value"] = not paused["value"]
        pause_button.config(text="ПРОДОЛЖИТЬ" if paused["value"] else "ПАУЗА")
        if not paused["value"]:
            # Поток мог быть погашен «стопом» или аварией — поднимаем.
            loop.ensure_running()
        # Движение разрешено ровно тогда, когда снята пауза. Раньше контур
        # правил положение всегда, и ось могла поехать сама.
        loop.set_armed(not paused["value"])
        if paused["value"]:
            # Пауза обязана именно останавливать. Замораживать цель мало:
            # старая цель осталась бы в силе, и ось продолжила бы ехать.
            loop.freeze()

    def emergency_stop():
        """Остановить движение, но не убивать программу.

        Раньше здесь гасился поток контура, и поднять его было нечем —
        после «стопа» приходилось перезапускать всё приложение. Теперь
        движение просто запрещается, а оси встают там, где стоят.
        """
        paused["value"] = True
        relaxed["value"] = False
        loop.set_armed(False)
        loop.freeze()
        pause_button.config(text="ПРОДОЛЖИТЬ")
        relax_button.config(text="РАССЛАБИТЬ")
        status.config(text="ОСТАНОВЛЕНО. Оси стоят под током. "
                           "«ПРОДОЛЖИТЬ» — снова поехать.",
                      foreground="#e05252")

    def toggle_diag():
        """Показать или спрятать строку со статистикой распознавания.

        В обычной работе она только мешает: проценты отброшенных кадров
        нужны при разборе, а не когда всё едет.
        """
        show_diag["value"] = not show_diag["value"]
        if show_diag["value"]:
            detector.pack(pady=(0, 8))
            diag_button.config(text="скрыть диагностику")
        else:
            detector.pack_forget()
            diag_button.config(text="диагностика")

    def capture_home() -> None:
        """Запомнить, где оси стоят сейчас, как исходное положение.

        Хранится в ИМПУЛЬСАХ энкодера, а не в градусах от программной
        середины: середину человек задаёт заново по многу раз за сеанс, и
        привязанное к ней исходное уезжало бы вместе с ней.
        """
        home.clear()
        for axis in axes:
            follower = loop.followers[axis.name]
            home[axis.name] = follower.telemetry.raw_counts
        save_home(home)
        home_button.config(text="В ИСХОДНОЕ")
        status.config(
            text="исходное положение запомнено. Кнопка теперь возвращает "
                 "в него; чтобы задать заново — «РАССЛАБИТЬ» и снова сюда.",
            foreground="")

    def go_home() -> None:
        """Вернуть оси в запомненное положение.

        Слежение за рукой при этом выключается: два источника цели
        одновременно тянули бы ось в разные стороны.
        """
        if not home:
            capture_home()
            return

        paused["value"] = True          # камера больше не ставит цели
        pause_button.config(text="ПРОДОЛЖИТЬ")
        loop.ensure_running()

        for axis in axes:
            follower = loop.followers[axis.name]
            follower.set_target(follower.counts_to_degrees(home[axis.name]))
        loop.set_armed(True)            # но контур ехать обязан
        state["homing"] = True
        status.config(text="возвращаюсь в исходное...", foreground="#ffb648")
        root.after(200, watch_homing)

    def watch_homing() -> None:
        """Довести возврат до конца и остановиться."""
        if not state.get("homing"):
            return
        worst = max(abs(loop.followers[a.name].telemetry.error_deg)
                    for a in axes)
        if worst > 2.0 and not loop.error_text:
            root.after(200, watch_homing)
            return
        state["homing"] = False
        loop.set_armed(False)
        loop.freeze()
        status.config(
            text=("возврат прерван: " + loop.error_text) if loop.error_text
            else "в исходном положении. «ПРОДОЛЖИТЬ» — снова за рукой",
            foreground="#e05252" if loop.error_text else "")

    def home_pressed() -> None:
        # Пока момент снят, человек ставит руку сам — значит хочет задать
        # исходное, а не ехать в него.
        if relaxed["value"] or not home:
            capture_home()
        else:
            go_home()

    def clear_fault():
        """Снять аварию и вернуть контур в работу — но только на паузе."""
        if not loop.clear_fault():
            status.config(text="аварии нет — сбрасывать нечего", foreground="")
            return
        paused["value"] = True
        pause_button.config(text="ПРОДОЛЖИТЬ")
        status.config(text="авария снята, оси стоят на месте. Проверьте "
                           "причину, потом снимайте паузу.", foreground="")

    def toggle_relax():
        """Снять момент, чтобы двигать руку робота вручную, и вернуть обратно."""
        relaxed["value"] = not relaxed["value"]
        loop.set_relaxed(relaxed["value"])
        relax_button.config(text="ЗАЖАТЬ" if relaxed["value"] else "РАССЛАБИТЬ")

        if relaxed["value"]:
            # Пауза обязана включиться: иначе при возврате момента ось
            # поедет к цели, которую источник наставил, пока руку двигали.
            paused["value"] = True
            pause_button.config(text="ПРОДОЛЖИТЬ")
            status.config(text="МОМЕНТ СНЯТ — двигайте руку робота руками. "
                               "Положение в таблице обновляется.",
                          foreground="#ffb648")
        else:
            status.config(text="момент возвращён. Выставьте середину заново, "
                               "если сдвигали оси, и снимайте паузу.",
                          foreground="")

    def set_middle():
        """Принять текущее положение и руки, и осей за середину."""
        current = arm_filter.last
        if current is None:
            status.config(text="руку не вижу — нечего принимать за середину",
                          foreground="#ffb648")
            return
        zero["q1"], zero["q2"] = current
        loop.set_zero_here()
        zeroed["value"] = True

        saved = {name: follower.zero_offset_counts
                 for name, follower in loop.followers.items()}
        save_calibration(saved)

        status.config(text=f"середина выставлена: плечо {zero['q1']:.0f}°, "
                           f"локоть {zero['q2']:.0f}°. Сохранено, "
                           f"можно снимать паузу.", foreground="")

    def use_saved():
        """Взять середину осей из файла, а позу руки — текущую."""
        current = arm_filter.last
        if current is None:
            status.config(text="руку не вижу — покажите её в кадре",
                          foreground="#ffb648")
            return
        saved = load_calibration()
        missing = [n for n in loop.followers if n not in saved]
        if missing:
            status.config(text=f"нет сохранённой середины для: "
                               f"{', '.join(missing)}", foreground="#ffb648")
            return
        for name, follower in loop.followers.items():
            follower.zero_offset_counts = int(saved[name])
        zero["q1"], zero["q2"] = current
        zeroed["value"] = True
        status.config(text="середина осей взята из файла, поза руки — "
                           "текущая. Можно снимать паузу.", foreground="")

    pause_button = ttk.Button(buttons, text="ПРОДОЛЖИТЬ", command=toggle_pause)
    pause_button.grid(row=0, column=0, padx=6)
    ttk.Button(buttons, text="СТОП", command=emergency_stop).grid(
        row=0, column=1, padx=6)
    ttk.Button(buttons, text="СЕРЕДИНА ЗДЕСЬ", command=set_middle).grid(
        row=0, column=2, padx=6)
    ttk.Button(buttons, text="взять сохранённую", command=use_saved).grid(
        row=0, column=3, padx=6)
    relax_button = ttk.Button(buttons, text="РАССЛАБИТЬ", command=toggle_relax)
    relax_button.grid(row=0, column=4, padx=6)
    ttk.Button(buttons, text="СБРОС ОШИБКИ", command=clear_fault).grid(
        row=0, column=5, padx=6)
    home_button = ttk.Button(buttons, command=home_pressed,
                             text="В ИСХОДНОЕ" if home else "ЗАПОМНИТЬ ИСХОДНОЕ")
    home_button.grid(row=0, column=6, padx=6)
    diag_button = ttk.Button(buttons, text="диагностика",
                             command=lambda: toggle_diag())
    diag_button.grid(row=0, column=7, padx=6)

    state = {"errors": 0, "last_error": "", "shown_frame": -1,
             "last_target_ms": 0.0}

    started_at = time.monotonic()
    journal = journal_file = None

    # Журнал пишется ВСЕГДА. Спорные вещи — «ноль уплыл», «ось не пошла» —
    # случаются редко и по памяти не восстанавливаются, а включать запись
    # заранее никто не догадывается: беда уже случилась.
    record_path = args.record
    if record_path is None and not args.no_record:
        folder = Path(__file__).resolve().parents[1] / "logs"
        folder.mkdir(exist_ok=True)
        record_path = folder / time.strftime("arm_%Y%m%d_%H%M%S.csv")

    if record_path:
        import csv
        journal_file = open(record_path, "w", newline="", encoding="utf-8")
        columns = ["t", "событие", "обмен_мс",
                   "рука_q1", "рука_q2", "отн_q1", "отн_q2"]
        for axis in axes:
            short = axis.name.split()[0]
            columns += [f"вал_{short}", f"сустав_{short}",
                        f"состояние_{short}"]
        journal = csv.DictWriter(journal_file, fieldnames=columns,
                                 extrasaction="ignore")
        journal.writeheader()
        log.info("журнал: %s", record_path)
    zeroed = {"value": False}
    home = load_home()
    relaxed = {"value": False}
    show_diag = {"value": False}

    def record(kind: str, **fields) -> None:
        """Записать строку в журнал, если он включён.

        Журнал нужен ровно для того, чтобы спорные вещи вроде «ноль
        уплывает» и «иногда перебегает» разбирать по числам. По памяти и
        на глаз такие вещи не ловятся: они редкие и накапливаются.
        """
        if journal is None:
            return
        journal.writerow({
            "t": f"{time.monotonic() - started_at:.3f}",
            "событие": kind,
            **{k: (f"{v:.3f}" if isinstance(v, float) else v)
               for k, v in fields.items()},
        })

    def on_target(smoothed):
        """Вызывается из потока распознавания."""
        state["last_target_ms"] = time.monotonic()
        if paused["value"] or not zeroed["value"]:
            return
        relative = (wrap180(smoothed[0] - zero["q1"]),
                    wrap180(smoothed[1] - zero["q2"]))
        loop.set_targets({
            axis.name: axis.motor_target(
                relative[0] if axis.joint == "q1" else relative[1])
            for axis in axes
        })
        record("цель",
               **{f"состояние_{a.name.split()[0]}":
                  ("расслаблена" if loop.followers[a.name].relaxed else
                   "запрещено" if not loop.followers[a.name].armed else
                   "упор" if loop.followers[a.name].out_of_range else
                   (loop.followers[a.name].telemetry.hold_reason or "удержание")
                   if loop.followers[a.name].telemetry.holding
                   else "едет")
                  for a in axes},
               **{f"обмен_мс": loop.sweep_ms},
               рука_q1=smoothed[0], рука_q2=smoothed[1],
               отн_q1=relative[0], отн_q2=relative[1],
               **{f"вал_{a.name.split()[0]}": loop.followers[a.name].telemetry.actual_deg
                  for a in axes},
               **{f"сустав_{a.name.split()[0]}":
                  a.motor_to_joint(loop.followers[a.name].telemetry.actual_deg)
                  for a in axes})

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
            if not zeroed["value"]:
                joint_state = "нет нуля"
            elif paused["value"] or relaxed["value"] or loop.error_text:
                joint_state = "стоп"
            else:
                joint_state = "работа"
            painted = draw_arm(cv2, frame, points, angles,
                               zero["q1"] if zeroed["value"] else None,
                               state=joint_state)
            image = Image.fromarray(painted)
            image.thumbnail((720, 430))
            photo = ImageTk.PhotoImage(image)
            keep["frame"] = photo
            video.configure(image=photo, text="")

        human_joints = None
        if smoothed is not None and zeroed["value"]:
            human_joints = (wrap180(smoothed[0] - zero["q1"]),
                            wrap180(smoothed[1] - zero["q2"]))

        commanded = []
        for axis in axes:
            telemetry = loop.followers[axis.name].telemetry
            labels = rows[axis.name]
            # Показываем сустав, а не вал: за валом стоит редуктор, и его
            # градусы человеку ни о чём не говорят.
            labels[3].config(text=f"{axis.motor_to_joint(telemetry.target_deg):7.1f}°")
            labels[4].config(text=f"{axis.motor_to_joint(telemetry.actual_deg):7.1f}°")
            labels[5].config(text=f"{axis.motor_to_joint(telemetry.error_deg):+7.1f}°")
            labels[6].config(text=f"{telemetry.command_rpm:+3d}")
            # Абсолютный угол привода — тот же, что показывает штатная
            # программа. По нему задаётся механический упор; без него его
            # пришлось бы подбирать вслепую.
            absolute = loop.followers[axis.name].absolute_angle(
                telemetry.raw_counts)
            labels[7].config(text="—" if absolute is None
                             else f"{absolute:7.1f}°")
            # Показываем ПРИЧИНУ, а не факт. «Удержание» означало сразу
            # несколько разных вещей, и при ошибке в сотню градусов
            # выглядело поломкой, хотя ось честно стояла на упоре.
            if relaxed["value"]:
                labels[8].config(text="расслаблена", foreground="#ffb648")
            elif telemetry.holding:
                reason = telemetry.hold_reason or "стоит"
                labels[8].config(
                    text=reason,
                    foreground="#e05252" if "упор" in reason else "")
            else:
                labels[8].config(text="едет", foreground="")

            out = loop.followers[axis.name].out_of_range
            clamped = False
            if human_joints is not None:
                raw = (human_joints[0] if axis.joint == "q1"
                       else human_joints[1])
                clamped = axis.is_clamped(raw)
            if out:
                text = f"ЗА ПРЕДЕЛОМ на {axis.motor_to_joint(out):.1f}°"
                colour = "#e05252"
            elif clamped:
                text = f"УПЁРЛАСЬ  {axis.limits_text()}"
                colour = "#ffb648"
            else:
                text, colour = axis.limits_text(), ""
            labels[9].config(text=text, foreground=colour)
            commanded.append(axis.motor_to_joint(telemetry.actual_deg))

        robot_view.update(commanded[0], commanded[1], human_joints)

        if loop.error_text:
            paused["value"] = True
            pause_button.config(text="ПРОДОЛЖИТЬ")
            status.config(text=loop.error_text, foreground="#e05252")
        elif loop.warning_text:
            status.config(text=loop.warning_text, foreground="#ffb648")
        elif not zeroed["value"]:
            status.config(text="сначала «СЕРЕДИНА ЗДЕСЬ» — без неё "
                               "приводы не поедут", foreground="#ffb648")
        elif (not paused["value"] and zeroed["value"]
              and time.monotonic() - state["last_target_ms"] > 2.0):
            # Самый обидный отказ: всё запущено, приводы отвечают, а руку
            # не видно — цели не обновляются, и вал стоит. Молчать об этом
            # нельзя, снаружи это неотличимо от поломки.
            # Причин у молчания несколько, и лечатся они по-разному.
            # Показать «не вижу руку», когда скелет на экране нарисован, —
            # значит заставить искать не там.
            counts = source.funnel.counts
            if counts["кадров"] == 0:
                reason = "кадры с камеры не идут"
            elif counts["поза найдена"] == 0:
                reason = ("камера видит, но позу не находит — модель не "
                          "распознаёт человека под этим ракурсом")
            elif counts["точки видны"] == 0:
                reason = f"поза есть, но сустав скрыт: {source.funnel.last_loss}"
            else:
                idle = time.monotonic() - state["last_target_ms"]
                reason = f"точек нет уже {idle:.0f} с"
            status.config(text=f"РОБОТ СТОИТ: {reason}", foreground="#e05252")
        elif relaxed["value"]:
            status.config(text="МОМЕНТ СНЯТ — оси свободны, положение "
                               "обновляется", foreground="#ffb648")
        elif paused["value"]:
            status.config(text="ПАУЗА — приводы не двигаются", foreground="#ffb648")
        else:
            status.config(text=f"ведём   обход шины {loop.sweep_ms:.0f} мс   "
                               f"всего {loop.sweeps}", foreground="")

        # Голос: слышно ли вообще, и что услышано последним. Без этого
        # «не работает» неотличимо от «не расслышал».
        if voice is None:
            voice_label.config(text="голос выключен", foreground="#666")
        elif voice.status == "нет доступа":
            voice_label.config(
                text=f"МИКРОФОН НЕДОСТУПЕН — {voice.error or ''}",
                foreground="#e05252")
        else:
            since = time.monotonic() - voice.last_heard_at
            if voice.last_heard and since < 3.0:
                voice_label.config(text=f"услышано: {voice.last_heard.upper()}",
                                   foreground="#3ddc84")
            else:
                bar = "▁▂▃▄▅▆▇█"
                filled = min(len(bar) - 1,
                             int(voice.level / max(args.voice_level, 1) * 4))
                tail = (f"  отброшено {voice.rejected}"
                        if voice.rejected else "")
                voice_label.config(
                    text=f"голос {voice.status}  {bar[filled]}  "
                         f"команд {voice.heard}{tail}",
                    foreground="#888")

        if not show_diag["value"]:
            return
        note = source.funnel.last_loss or source.note
        errors = state["errors"] + worker.errors
        tail = ""
        if errors:
            tail = f"   СБОЕВ {errors}: {worker.last_error or state['last_error']}"
        detector.config(
            text=f"распознавание {worker.fps:.0f} к/с ({worker.poll_ms:.0f} мс)   "
                 f"{source.funnel.report()}   |   {note}{tail}",
            foreground="#e05252" if errors else "#888")

    def on_voice(command: str) -> None:
        """Голосовая команда. Приходит из потока распознавания.

        Сам обработчик ничего не делает: он передаёт вызов в поток окна
        через root.after. Трогать интерфейс Tk из чужого потока нельзя —
        он от этого падает не сразу, а позже и в непонятном месте.
        """
        actions = {"стоп": emergency_stop,
                   "старт": lambda: (toggle_pause() if paused["value"] else None),
                   "линия": set_middle,
                   "положение": home_pressed}
        action = actions.get(command)
        if action is not None:
            root.after(0, action)

    voice = None
    if not args.no_voice and os.environ.get("SUDO_LAUNCH"):
        log.warning("запуск под sudo: микрофон macOS обычно не отдаёт "
                    "процессу от root. Если голос не заработает, "
                    "запускайте без --realsense и без sudo, либо "
                    "пользуйтесь кнопками")
    if not args.no_voice:
        try:
            from scara_drive.voice import VoiceCommands
            voice = VoiceCommands(on_voice, device=args.audio_device,
                                  silence_level=args.voice_level)
            voice.bias = args.voice_bias
            voice.start()
        except SystemExit as exc:
            log.warning("голосовые команды выключены: %s",
                        str(exc).splitlines()[0])
            voice = None

    def shutdown():
        if voice is not None:
            voice.stop()
        if journal_file is not None:
            journal_file.close()
            log.info("журнал закрыт: %s", record_path)
        worker.stop()
        try:
            loop.set_relaxed(True)      # не оставляем оси под током
        except Exception:
            log.exception("не удалось снять момент при выходе")
        loop.stop()
        source.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", shutdown)
    loop.set_armed(False)          # окно открывается с запрещённым движением
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
    p.add_argument("--max-rpm", type=int, default=0,
                   help="общий потолок об/мин; 0 — считать по каждой оси")
    p.add_argument("--kp", type=float, default=0.6,
                   help="об/мин на градус ошибки; больше — плотнее слежение")
    p.add_argument("--ramp", type=float, default=0.8,
                   help="секунд до полной скорости; больше — плавнее")
    p.add_argument("--tolerance", type=float, default=1.0,
                   help="допуск в градусах СУСТАВА, одинаковый для всех осей")
    p.add_argument("--period", type=float, default=0.12)
    p.add_argument("--sweep", type=float, default=0.12)
    p.add_argument("--camera", type=int, default=0,
                   help="номер обычной камеры")
    p.add_argument("--allow-camera-reset", action="store_true",
                   help="разрешить сброс камеры при неудаче; она может "
                        "не вернуться и потребовать переподключения кабеля")
    p.add_argument("--realsense", action="store_true",
                   help="брать кадры с RealSense через SDK, а не как с "
                        "веб-камеры; нужен sudo и питон из arm_realsense")
    p.add_argument("--list-cameras", action="store_true",
                   help="показать доступные камеры и выйти")
    p.add_argument("--no-voice", action="store_true",
                   help="выключить голосовые команды")
    p.add_argument("--audio-device", type=int, default=0,
                   help="номер микрофона; список — tools/mics.py")
    p.add_argument("--voice-bias", type=float, default=0.0,
                   help="сдвиг порогов распознавания: минус — принимать "
                        "охотнее, плюс — строже")
    p.add_argument("--voice-level", type=int, default=300,
                   help="порог тишины: ниже него звук не распознаётся")
    p.add_argument("--record", metavar="ФАЙЛ.csv",
                   help="куда писать журнал; по умолчанию в logs/")
    p.add_argument("--no-record", action="store_true",
                   help="не вести журнал вовсе")
    p.add_argument("--median", type=int, default=3,
                   help="кадров в медианном окне; гасит срывы точек")
    p.add_argument("--min-cutoff", type=float, default=1.0,
                   help="сглаживание в покое; меньше — спокойнее, но вязче")
    p.add_argument("--beta", type=float, default=0.1,
                   help="как быстро сглаживание отпускает на движении")
    p.add_argument("--fixed-smoothing", action="store_true",
                   help="постоянное сглаживание вместо переменного")
    p.add_argument("--smoothing", type=float, default=0.6,
                   help="сила постоянного сглаживания, если включено")
    p.add_argument("--diag", action="store_true",
                   help="замерить, где теряются кадры, и выйти")
    p.add_argument("--diag-seconds", type=float, default=12.0)
    p.add_argument("--min-visibility", type=float, default=0.5,
                   help="порог видимости сустава, ниже — кадр отбрасывается")
    p.add_argument("--limit-margin", type=float, default=3.0,
                   help="допустимый перелёт за предел, градусов вала")
    p.add_argument("--runaway-passes", type=int, default=6,
                   help="проходов с растущей ошибкой до аварийной остановки")
    p.add_argument("--test-move", type=float, metavar="ГРАДУСЫ",
                   help="погонять оси без камеры на заданный угол сустава")
    p.add_argument("--test-seconds", type=float, default=4.0)
    p.add_argument("--skip-mode-check", action="store_true",
                   help="не проверять режим приводов перед запуском")
    p.add_argument("--skip-line-check", action="store_true",
                   help="не проверять шину на шум перед запуском")
    p.add_argument("--no-limits", action="store_true",
                   help="снять пределы осей: повторять руку без ограничений")
    p.add_argument("--gain", type=float,
                   help="переопределить усиление для всех осей")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(message)s")

    if args.list_cameras:
        shots = Path(__file__).resolve().parents[1] / "camera_shots"
        cameras = list_cameras(snapshot_dir=shots)
        if not cameras:
            print("камер не найдено.")
            print("на macOS доступ выдаётся приложению, из которого запущен")
            print("python, и подхватывается только НОВЫМ процессом:")
            print("  разрешите в Системные настройки -> Конфиденциальность")
            print("  и безопасность -> Камера, затем перезапустите Терминал.")
            return 1
        names = camera_names()
        print("доступные камеры:\n")
        for index, width, height, saved in cameras:
            line = f"  --camera {index}   {width}x{height}"
            if index < len(names):
                line += f"   {names[index]}"
            if saved:
                line += f"   [{saved.name}]"
            print(line)

        if names:
            print("\nимена от системы (порядок может не совпадать с "
                  "индексами):")
            for i, name in enumerate(names):
                print(f"  {i}. {name}")

        joined = " ".join(names).lower()
        if "iphone" in joined or "patron" in joined or "continuity" in joined:
            print("\nВНИМАНИЕ: в списке есть iPhone. macOS подставляет его")
            print("как камеру, когда телефон рядом, и НУМЕРАЦИЯ СДВИГАЕТСЯ —")
            print("та же цифра завтра означает другую камеру.")
            print("Отключите Continuity Camera на телефоне:")
            print("  Настройки -> Основные -> AirPlay и Continuity ->")
            print("  Continuity Camera — выключить")
        if "depth" in joined:
            print("\nВНИМАНИЕ: от RealSense виден узел Depth, а не RGB.")
            print("Это инфракрасный поток, а не цветной. Распознавание позы")
            print("рассчитано на цветной кадр и по ИК работает ненадёжно.")
            print("Посмотрите сохранённый кадр: если он серый — это оно.")
        if any(c[3] for c in cameras):
            print(f"\nкадры сохранены в {shots}")
            print("откройте их — по картинке видно, какая камера какая:")
            print(f"    open {shots}")
        return 0

    if not (args.simulate or args.confirm or args.diag
            or args.test_move is not None):
        print(__doc__)
        print("Запустите с --confirm, когда оси будут разгружены, "
              "с --simulate для проверки без железа "
              "или с --diag, чтобы померить распознавание.")
        return 1

    if args.diag:
        source = (PoseArmSource(args.camera, args.side,
                                realsense=args.realsense,
                                allow_reset=args.allow_camera_reset)
                  if args.source == "pose"
                  else MarkerArmSource(args.camera, realsense=args.realsense,
                                allow_reset=args.allow_camera_reset))
        source.MIN_VISIBILITY = args.min_visibility
        arm_filter = ArmFilter(median_window=args.median,
                               adaptive=not args.fixed_smoothing,
                               min_cutoff=args.min_cutoff, beta=args.beta,
                               smoothing=args.smoothing,
                               max_len_change=float("inf"),
                               max_jump_deg=float("inf"))
        try:
            return run_diag(source, arm_filter, args.diag_seconds)
        finally:
            source.close()

    axes = load_axes()
    if args.no_limits:
        for axis in axes:
            axis.min_deg = axis.max_deg = None
        log.warning("пределы осей сняты: программа их не удержит, "
                    "остаётся только механика и аварийный останов")
    if args.gain is not None:
        for axis in axes:
            axis.gain = args.gain

    print("оси:")
    for axis in axes:
        joint = "плечо" if axis.joint == "q1" else "локоть"
        print(f"  id={axis.device_id}  {axis.name:<11} <- {joint:<7} "
              f"{axis.gear_text():<22} сустав {axis.limits_text()}")
        print(f"           {'':<11}    это {axis.joint_to_motor(axis.max_deg):+.0f}° "
              f"вала, потолок {axis.max_motor_rpm} об/мин")
    print()
    for axis in axes:
        tol = axis_tolerance(args, axis)
        raised = "  (поднят: ниже ось рыскала бы)" if (
            tol > args.tolerance * abs(axis.gear_ratio) + 1e-6) else ""
        cfg = axis_config(args, axis)
        if axis.abs_zero_counts is not None:
            print(f"  упор {axis.name}: разрешена дуга "
                  f"{axis.abs_min_deg:.0f}..{axis.abs_max_deg:.0f}° "
                  f"по шкале привода (ноль {axis.abs_zero_counts})")
        print(f"  допуск {axis.name}: "
              f"{abs(axis.motor_to_joint(tol)):.2f}° сустава{raised}"
              f",  разгон {cfg.max_rpm_step:.1f} об/мин за проход")
    print()
    print("защита: цель прижимается к пределам; ось за пределом сама")
    print("        возвращается в зону; движение не в ту сторону гасит")
    print("        ОБЕ оси. Кнопка «СБРОС ОШИБКИ» снимает аварию.")
    print()

    followers, bus = build_followers(args, axes)
    loop = MultiAxisFollower(followers, period_s=args.sweep)
    loop.set_zero_here()

    if args.test_move is not None:
        try:
            return run_test_move(loop, axes, args.test_move, args.test_seconds)
        finally:
            if bus is not None:
                try:
                    bus.stop_all()
                finally:
                    bus.close()

    source = (PoseArmSource(args.camera, args.side, realsense=args.realsense,
                                allow_reset=args.allow_camera_reset)
              if args.source == "pose"
              else MarkerArmSource(args.camera, realsense=args.realsense,
                                allow_reset=args.allow_camera_reset))
    source.MIN_VISIBILITY = args.min_visibility
    # Кадры не отбрасываются: отсев резал нормальные движения, а картина
    # «скелет виден, робот стоит» разбиралась дольше, чем стоили пойманные
    # сбои. Шум давится двумя средствами, и роли у них разные.
    #
    # Медиана убирает СРЫВЫ — одиночные улетевшие точки. На замерах окно
    # из трёх кадров срезает худший выброс с 37° до 2.4°; окна больше
    # ничего не добавляют, только запаздывание.
    #
    # Переменное сглаживание убирает ДРОЖЬ — мелкое непрерывное
    # подрагивание. Оно давит сильно, пока рука почти стоит, и отпускает
    # на движении, поэтому не платит за тишину запаздыванием.
    arm_filter = ArmFilter(median_window=args.median,
                           adaptive=not args.fixed_smoothing,
                           min_cutoff=args.min_cutoff, beta=args.beta,
                           smoothing=args.smoothing,
                           max_len_change=float("inf"),
                           max_jump_deg=float("inf"))

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
