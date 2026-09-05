"""Слежение вала за рукой в реальном времени: слева 0°, справа 360°.

Источники цели:

  mouse   положение курсора по горизонтали в окне (ничего ставить не надо)
  camera  ладонь в кадре веб-камеры, нужен mediapipe

Запуск:

    python tools/hand_follow.py --simulate          проверка без привода
    python tools/hand_follow.py --confirm           на живом приводе
    python tools/hand_follow.py --confirm --source camera

Проверка логики контура без окна и без железа:

    python tools/hand_follow.py --headless --simulate

ВАЖНО: с --confirm скрипт КРУТИТ ВАЛ вслед за рукой. Ось должна быть
разгружена, аварийный останов под рукой. Закрытие окна останавливает привод.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import DriveConfig, SerialConfig, ServoDrive, TcpConfig
from scara_drive.discovery import guess_port
from scara_drive.simulation import SimulatedAxis as SimulatedDrive
from scara_drive.follow import (AngleFollower, FollowConfig, normalize_360,
                                shortest_angle_error)

log = logging.getLogger("hand_follow")


class MouseSource:
    """Цель из положения курсора по горизонтали внутри окна."""

    name = "мышь"
    hint = "веди курсор по окну: слева 0°, справа 360°"

    def bind(self, widget, on_angle) -> None:
        def handler(event):
            width = max(widget.winfo_width(), 1)
            fraction = min(max(event.x / width, 0.0), 1.0)
            on_angle(fraction * 360.0)

        widget.bind("<Motion>", handler)

    def close(self) -> None:
        pass


class CameraSource:
    """Цель из положения ладони в кадре веб-камеры.

    mediapipe 1.x: старого ``mp.solutions.hands`` больше нет, работает только
    Tasks API, и ему нужен файл модели ``hand_landmarker.task``.
    """

    name = "камера"
    hint = "двигай ладонью влево-вправо перед камерой"

    MODEL_NAME = "hand_landmarker.task"
    MODEL_URL = (
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
        "hand_landmarker/float16/1/hand_landmarker.task"
    )

    def __init__(self, camera_index: int = 0, model_path: Path | None = None):
        try:
            import cv2
            import mediapipe as mp
            from mediapipe.tasks import python as mp_tasks
            from mediapipe.tasks.python import vision
        except ImportError as exc:
            raise SystemExit(
                "для --source camera нужны mediapipe и opencv:\n"
                "    ./.venv/bin/pip install mediapipe\n"
                f"(не найдено: {exc.name})"
            ) from exc

        self._cv2 = cv2
        self._mp = mp

        model = model_path or (Path(__file__).resolve().parents[1]
                               / "models" / self.MODEL_NAME)
        if not model.is_file():
            raise SystemExit(
                f"нет файла модели: {model}\n\n"
                "mediapipe 1.x не поставляет модели вместе с пакетом.\n"
                "Скачайте её один раз (около 7.5 МБ):\n\n"
                f"    mkdir -p {model.parent}\n"
                f"    curl -L -o {model} \\\n        {self.MODEL_URL}\n"
            )

        options = vision.HandLandmarkerOptions(
            # Делегат строго CPU: на macOS сборка mediapipe 1.0.1 падает при
            # инициализации Metal ("graph_service: Service is unavailable"),
            # роняя весь процесс, а не выбрасывая исключение.
            base_options=mp_tasks.BaseOptions(
                model_asset_path=str(model),
                delegate=mp_tasks.BaseOptions.Delegate.CPU,
            ),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)

        self._capture = cv2.VideoCapture(camera_index)
        if not self._capture.isOpened():
            raise SystemExit(
                f"камера {camera_index} не открылась.\n\n"
                "На macOS доступ к камере даётся тому приложению, из которого\n"
                "запущен python (Терминал, iTerm, Claude Code). Порядок:\n"
                "  1. разрешить в диалоге, либо Системные настройки ->\n"
                "     Конфиденциальность и безопасность -> Камера;\n"
                "  2. перезапустить это приложение целиком — выданное\n"
                "     разрешение подхватывается только новым процессом;\n"
                "  3. запустить скрипт снова."
            )
        self._start = time.monotonic()
        self.seen_hand = False
        self.last_frame = None      # RGB-кадр для показа в окне
        self.last_hand_x = None     # доля ширины 0..1, где найдено запястье

    def poll(self) -> float | None:
        """Вернуть угол 0..360 или None, если руки в кадре нет."""
        ok, frame = self._capture.read()
        if not ok:
            return None

        frame = self._cv2.flip(frame, 1)      # зеркало: ведёшь вправо — вправо
        rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((time.monotonic() - self._start) * 1000)

        result = self._landmarker.detect_for_video(image, timestamp_ms)

        if not result.hand_landmarks:
            self.last_hand_x = None
            self.last_frame = rgb
            return None

        self.seen_hand = True
        landmarks = result.hand_landmarks[0]
        wrist = landmarks[0]                  # точка 0 — запястье
        x = min(max(wrist.x, 0.0), 1.0)
        self.last_hand_x = x
        self.last_frame = self._annotate(rgb, landmarks, x)
        return x * 360.0

    def _annotate(self, rgb, landmarks, x: float):
        """Отметить на кадре найденную кисть — чтобы видеть, что поймал детектор."""
        cv2 = self._cv2
        frame = rgb.copy()
        height, width = frame.shape[:2]

        for point in landmarks:
            cx = int(min(max(point.x, 0.0), 1.0) * width)
            cy = int(min(max(point.y, 0.0), 1.0) * height)
            cv2.circle(frame, (cx, cy), 3, (0, 220, 130), -1)

        line_x = int(x * width)
        cv2.line(frame, (line_x, 0), (line_x, height), (74, 158, 255), 2)
        return frame

    def close(self) -> None:
        self._capture.release()
        self._landmarker.close()


def build_drive(args):
    if args.simulate:
        return SimulatedDrive(counts_per_rev=args.cpr, invert=not args.no_invert)
    if args.host:
        config = DriveConfig(tcp=TcpConfig(host=args.host, device_id=args.id))
    else:
        port = args.port or guess_port()
        if port is None:
            raise SystemExit("не нашёл переходник — укажите --port")
        config = DriveConfig(serial=SerialConfig(
            port=port, baudrate=args.baud, parity=args.parity, device_id=args.id))
    drive = ServoDrive(config)
    drive.connect()
    return drive


def run_headless(follower: AngleFollower, seconds: float) -> int:
    """Прогнать контур по заданной траектории цели и напечатать, как шло."""
    print("проверка контура без окна: цель ходит слева направо и обратно\n")
    print(f"{'с':>5} {'цель':>7} {'факт':>7} {'ошибка':>8} {'об/мин':>7}  режим")
    print("-" * 48)

    start = time.monotonic()
    worst = 0.0
    with follower:
        while True:
            elapsed = time.monotonic() - start
            if elapsed > seconds:
                break
            # Треугольная развёртка 0 -> 360 -> 0 за 12 секунд.
            phase = (elapsed % 12.0) / 12.0
            target = phase * 720.0 if phase < 0.5 else (1.0 - phase) * 720.0
            follower.set_target(target)

            t = follower.telemetry
            worst = max(worst, abs(t.error_deg))
            print(f"{elapsed:5.1f} {t.target_deg:7.1f} {t.actual_deg:7.1f} "
                  f"{t.error_deg:8.2f} {t.command_rpm:7d}  "
                  f"{'удержание' if t.holding else 'едет'}")
            time.sleep(0.5)

    print(f"\nпроходов контура: {follower.telemetry.passes}")
    print(f"наибольшее отставание от цели: {worst:.1f}°")
    if follower.telemetry.error_text:
        print(f"контур упал: {follower.telemetry.error_text}")
        return 1
    return 0


def run_gui(follower: AngleFollower, source, args) -> int:
    import tkinter as tk
    from tkinter import ttk

    show_camera = isinstance(source, CameraSource)
    photo_holder = {}          # держим ссылку на PhotoImage, иначе Tk её съест

    if show_camera:
        try:
            from PIL import Image, ImageTk
        except ImportError:
            log.warning("нет Pillow — окно будет без картинки с камеры")
            show_camera = False

    root = tk.Tk()
    root.title(f"Слежение за рукой — {source.name}")
    root.minsize(820, 660 if show_camera else 320)

    header = ttk.Label(root, text=source.hint, font=("SF Pro Text", 13, "bold"))
    header.pack(pady=(14, 6))

    if show_camera:
        video = tk.Label(root, background="#111", text="ожидание кадра...",
                         foreground="#777", font=("SF Pro Text", 12))
        video.pack(padx=16, pady=(0, 8))

    canvas = tk.Canvas(root, height=130, background="#1b1b1b", highlightthickness=0)
    canvas.pack(fill="both", expand=True, padx=16, pady=8)

    readout = ttk.Label(root, text="—", font=("SF Pro Text", 12))
    readout.pack(pady=(0, 4))
    status = ttk.Label(root, text="", font=("SF Pro Text", 11))
    status.pack(pady=(0, 12))

    if isinstance(source, MouseSource):
        source.bind(canvas, follower.set_target)

    def draw_video() -> None:
        frame = getattr(source, "last_frame", None)
        if frame is not None:
            image = Image.fromarray(frame)
            image.thumbnail((640, 400))
            photo = ImageTk.PhotoImage(image)
            photo_holder["frame"] = photo        # ссылка обязательна
            video.configure(image=photo, text="")
        root.after(40, draw_video)

    def draw() -> None:
        t = follower.telemetry
        width = max(canvas.winfo_width(), 1)
        canvas.delete("all")

        for fraction, label in ((0.0, "0°"), (0.5, "180°"), (1.0, "360°")):
            x = fraction * (width - 2) + 1
            canvas.create_line(x, 96, x, 112, fill="#555")
            canvas.create_text(x, 122, text=label, fill="#888",
                               anchor="n" if fraction else "nw")

        target_x = t.target_deg / 360.0 * width
        actual_x = t.actual_deg / 360.0 * width
        canvas.create_line(target_x, 10, target_x, 92, fill="#4a9eff", width=3)
        canvas.create_text(target_x, 4, text="рука", fill="#4a9eff", anchor="n")
        canvas.create_line(actual_x, 30, actual_x, 92,
                           fill="#3ddc84" if t.holding else "#ffb648", width=5)
        canvas.create_text(actual_x, 46, text="вал", fill="#fff", anchor="n")

        readout.config(
            text=f"рука {t.target_deg:6.1f}°     вал {t.actual_deg:6.1f}°     "
                 f"ошибка {t.error_deg:+6.1f}°     {t.command_rpm:+3d} об/мин"
        )

        if t.error_text:
            status.config(text=f"ОСТАНОВЛЕНО: {t.error_text}", foreground="#e05252")
        elif t.warning_text:
            status.config(text=t.warning_text, foreground="#ffb648")
        else:
            hand = ""
            if show_camera and getattr(source, "last_hand_x", None) is None:
                hand = "   руки в кадре нет"
            dropped = f"   сбоев {t.dropped_passes}" if t.dropped_passes else ""
            status.config(
                text=f"{'удержание' if t.holding else 'едет'}   "
                     f"проход {t.period_ms:.0f} мс   всего {t.passes}"
                     f"{dropped}{hand}",
                foreground="",
            )
        root.after(50, draw)

    def poll_camera() -> None:
        angle = source.poll()
        if angle is not None:
            follower.set_target(angle)
        root.after(30, poll_camera)

    def shutdown() -> None:
        follower.stop()
        source.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", shutdown)
    follower.start()
    draw()
    if isinstance(source, CameraSource):
        poll_camera()
        if show_camera:
            draw_video()

    try:
        root.mainloop()
    finally:
        follower.stop()
        source.close()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=["mouse", "camera"], default="mouse")
    p.add_argument("--simulate", action="store_true", help="привод на бумаге")
    p.add_argument("--headless", action="store_true", help="без окна, самопроверка")
    p.add_argument("--confirm", action="store_true",
                   help="подтвердить, что ось разгружена (нужно без --simulate)")
    p.add_argument("--seconds", type=float, default=15.0, help="для --headless")

    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--parity", default="E", choices=["N", "E", "O"])
    p.add_argument("--id", type=int, default=1)
    p.add_argument("--host")

    p.add_argument("--cpr", type=int, default=131072)
    p.add_argument("--no-invert", action="store_true",
                   help="энкодер считает в ту же сторону, что и команда")
    p.add_argument("--max-rpm", type=int, default=20)
    p.add_argument("--kp", type=float, default=0.225)
    p.add_argument("--tolerance", type=float, default=2.0)
    p.add_argument("--period", type=float, default=0.08)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(message)s")

    if not args.simulate and not args.confirm:
        print(__doc__)
        print("Запустите с --confirm, когда ось будет разгружена, "
              "или с --simulate для проверки без железа.")
        return 1

    config = FollowConfig(
        counts_per_rev=args.cpr, invert_encoder=not args.no_invert,
        kp=args.kp, max_rpm=args.max_rpm, tolerance_deg=args.tolerance,
        period_s=args.period,
    )
    warning = config.check_tolerance()
    if warning:
        log.warning(warning)

    drive = build_drive(args)
    follower = AngleFollower(drive, config)
    follower.set_zero_here()
    log.info("текущее положение принято за 0°")

    source = MouseSource() if args.source == "mouse" else CameraSource()
    try:
        if args.headless:
            return run_headless(follower, args.seconds)
        return run_gui(follower, source, args)
    finally:
        try:
            drive.stop()
        finally:
            drive.close()


if __name__ == "__main__":
    raise SystemExit(main())
