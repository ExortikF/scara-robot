"""Управление тремя осями SCARA рукой перед камерой.

Перенос старой системы (School_etc/cv_control.py) с сервоприводов ST3215 на
приводы C30 по Modbus RTU. Что сохранено из оригинала:

* та же схема отображения точек кисти в оси — X в базу, Y в плечо, Z в локоть;
* те же входные диапазоны landmark (0.2..0.8 по X и Y, -0.2..0.2 по Z);
* экспоненциальное сглаживание цели;
* пределы по каждой оси, за которые выйти нельзя;
* при пропаже руки из кадра цели замирают, а не прыгают в середину.

Что изменилось из-за железа:

* адресация по slave id (1, 2, 3) на одной шине RS-485 вместо шины ST3215;
* привод не принимает позицию, поэтому под каждой осью работает контур
  слежения поверх скоростного режима (docs/why_not_modbus_positioning.md);
* три оси на одной шине обслуживаются по очереди в одном потоке.

Запуск:

    python tools/scara_cv_control.py --simulate     проверка без приводов
    python tools/scara_cv_control.py --confirm      на живых приводах

ВАЖНО: с --confirm приводы КРУТЯТСЯ вслед за рукой. Оси должны быть
разгружены, аварийный останов под рукой. В окне есть кнопка СТОП.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive import DriveBus, SerialConfig, TcpConfig
from scara_drive.discovery import guess_port
from scara_drive.simulation import SimulatedAxis
from scara_drive.follow import AngleFollower, FollowConfig, MultiAxisFollower

log = logging.getLogger("scara_cv")

AXES_FILE = Path(__file__).with_name("scara_axes.json")


@dataclass
class AxisSpec:
    """Одна ось: куда подключена, чем управляется, куда ей можно.

    ``span_deg`` — полный ход относительно положения при запуске. Ноль
    ставится там, где ось стоит в момент старта, поэтому рука в середине
    кадра означает «не двигаться». Так первый запуск не может увести ось
    далеко от исходной точки.
    """

    name: str
    device_id: int
    hand: str            # "Right" | "Left" — с какой руки берём координату
    source: str          # "x" | "y" — координата запястья в кадре (2D)
    in_min: float
    in_max: float
    span_deg: float
    invert: bool = False

    def target_deg(self, value: float) -> float:
        """Отобразить координату landmark в смещение от нуля, в градусах."""
        span = self.in_max - self.in_min
        fraction = (value - self.in_min) / span if span else 0.5
        fraction = min(max(fraction, 0.0), 1.0)
        if self.invert:
            fraction = 1.0 - fraction
        return (fraction - 0.5) * self.span_deg


DEFAULT_AXES = [
    # Диапазоны landmark — из оригинального cv_control.py.
    # Первая рука ведёт две оси, вторая — третью. Только X и Y: координата Z
    # у mediapipe оценочная и заметно шумит, в 2D она не нужна.
    AxisSpec("J1 база",   1, "Right", "x", 0.2, 0.8, span_deg=90.0),
    AxisSpec("J2 плечо",  2, "Right", "y", 0.2, 0.8, span_deg=60.0, invert=True),
    AxisSpec("J3 локоть", 3, "Left",  "y", 0.2, 0.8, span_deg=40.0, invert=True),
]


def load_axes() -> list[AxisSpec]:
    """Прочитать описание осей из файла рядом со скриптом, либо взять умолчания."""
    if not AXES_FILE.is_file():
        AXES_FILE.write_text(
            json.dumps([asdict(a) for a in DEFAULT_AXES], ensure_ascii=False,
                       indent=2),
            encoding="utf-8",
        )
        log.info("создан %s — правьте пределы там", AXES_FILE.name)
        return DEFAULT_AXES
    data = json.loads(AXES_FILE.read_text(encoding="utf-8"))
    return [AxisSpec(**item) for item in data]


class HandSource:
    """Кисть в кадре: отдаёт координаты запястья и сам кадр для показа."""

    MODEL_NAME = "hand_landmarker.task"
    MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
                 "hand_landmarker/float16/1/hand_landmarker.task")

    #: Кисти различаются по метке mediapipe. Кадр зеркалим для показа, из-за
    #: чего метки могут оказаться перевёрнуты относительно ощущений — на этот
    #: случай есть --swap-hands.
    HANDS = ("Right", "Left")

    def __init__(self, camera_index: int = 0, model_path: Path | None = None,
                 swap_hands: bool = False):
        self.swap_hands = swap_hands
        try:
            import cv2
            import mediapipe as mp
            from mediapipe.tasks import python as mp_tasks
            from mediapipe.tasks.python import vision
        except ImportError as exc:
            raise SystemExit(
                "нужны mediapipe и opencv:\n"
                "    ./.venv/bin/pip install -r requirements.txt\n"
                f"(не найдено: {exc.name})"
            ) from exc

        self._cv2, self._mp = cv2, mp
        model = model_path or (Path(__file__).resolve().parents[1]
                               / "models" / self.MODEL_NAME)
        if not model.is_file():
            raise SystemExit(
                f"нет файла модели: {model}\n\n"
                "Скачайте один раз (около 7.5 МБ):\n"
                f"    mkdir -p {model.parent}\n"
                f"    curl -L -o {model} \\\n        {self.MODEL_URL}\n"
            )

        options = vision.HandLandmarkerOptions(
            # Строго CPU: на macOS Metal-делегат mediapipe роняет процесс.
            base_options=mp_tasks.BaseOptions(
                model_asset_path=str(model),
                delegate=mp_tasks.BaseOptions.Delegate.CPU),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)

        self._capture = cv2.VideoCapture(camera_index)
        if not self._capture.isOpened():
            raise SystemExit(
                f"камера {camera_index} не открылась.\n\n"
                "На macOS доступ выдаётся приложению, из которого запущен\n"
                "python, и подхватывается только НОВЫМ процессом:\n"
                "  1. разрешить в диалоге или Системные настройки ->\n"
                "     Конфиденциальность и безопасность -> Камера;\n"
                "  2. перезапустить Терминал целиком;\n"
                "  3. запустить скрипт снова."
            )
        self._start = time.monotonic()
        self.last_frame = None
        self.hands: dict[str, tuple[float, float]] = {}   # метка -> (x, y)

    def poll(self) -> dict[str, tuple[float, float]]:
        """Вернуть {"Right": (x, y), "Left": (x, y)} по видимым кистям."""
        ok, frame = self._capture.read()
        if not ok:
            return self.hands

        frame = self._cv2.flip(frame, 1)        # зеркало
        rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(
            image, int((time.monotonic() - self._start) * 1000))

        found: dict[str, tuple[float, float]] = {}
        drawable = []

        for index, landmarks in enumerate(result.hand_landmarks):
            label = self._label(result, index)
            wrist = landmarks[0]
            found[label] = (wrist.x, wrist.y)
            drawable.append((label, landmarks, wrist))

        self.hands = found
        self.last_frame = self._draw(rgb, drawable)
        return found

    def _label(self, result, index: int) -> str:
        """Метка кисти. При неудаче — по порядку обнаружения."""
        try:
            label = result.handedness[index][0].category_name
        except (AttributeError, IndexError):
            label = self.HANDS[index % len(self.HANDS)]
        if self.swap_hands:
            label = "Left" if label == "Right" else "Right"
        return label

    COLORS = {"Right": (74, 158, 255), "Left": (255, 176, 72)}

    def _draw(self, rgb, drawable):
        cv2 = self._cv2
        frame = rgb.copy()
        height, width = frame.shape[:2]

        for label, landmarks, wrist in drawable:
            color = self.COLORS.get(label, (200, 200, 200))
            for point in landmarks:
                cx = int(min(max(point.x, 0.0), 1.0) * width)
                cy = int(min(max(point.y, 0.0), 1.0) * height)
                cv2.circle(frame, (cx, cy), 3, (0, 220, 130), -1)

            wx = int(min(max(wrist.x, 0.0), 1.0) * width)
            wy = int(min(max(wrist.y, 0.0), 1.0) * height)
            cv2.line(frame, (wx, 0), (wx, height), color, 2)
            cv2.line(frame, (0, wy), (width, wy), color, 2)
            cv2.circle(frame, (wx, wy), 9, (255, 255, 255), 2)
            cv2.putText(frame, label, (wx + 14, wy - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        return frame

    def close(self):
        self._capture.release()
        self._landmarker.close()


def preflight_line(transport: SerialConfig) -> None:
    """Убедиться, что линия молчит, пока молчим мы.

    Без этой проверки шумящая шина приводит не к внятной ошибке, а к
    зависанию: поток мусора не даёт чтению завершиться по таймауту.
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

    raise SystemExit(
        f"\nЛИНИЯ ШУМИТ: с молчащей шины пришло {noise} байт за 0.5 с.\n\n"
        "Работать так нельзя — обмен будет зависать: пока байты идут\n"
        "непрерывно, чтение ответа не завершается по таймауту.\n\n"
        "Две обычные причины:\n"
        "  1. на шине второй ведущий — закройте другие программы,\n"
        "     работающие с этим портом (в том числе C30 Servo GUI);\n"
        "  2. линия без смещения и терминаторов — см. подробности:\n"
        "     python tools/bus_health.py\n\n"
        "Запустить вопреки проверке: --skip-line-check\n"
    )


def build_followers(args, specs: list[AxisSpec]):
    """Собрать контуры по осям. Возвращает (followers, bus | None)."""
    config = FollowConfig(
        counts_per_rev=args.cpr,
        invert_encoder=not args.no_invert,
        kp=args.kp,
        max_rpm=args.max_rpm,
        tolerance_deg=args.tolerance,
        period_s=args.period,
    )
    warning = config.check_tolerance()
    if warning:
        log.warning(warning)

    if args.simulate:
        followers = {
            spec.name: AngleFollower(
                SimulatedAxis(args.cpr, not args.no_invert), config)
            for spec in specs
        }
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

    followers = {spec.name: AngleFollower(bus.axis(spec.device_id), config)
                 for spec in specs}
    return followers, bus


def run_gui(loop: MultiAxisFollower, source: HandSource,
            specs: list[AxisSpec], args) -> int:
    import tkinter as tk
    from tkinter import ttk
    from PIL import Image, ImageTk

    photo_holder = {}
    paused = {"value": False}

    root = tk.Tk()
    root.title("SCARA — управление рукой")
    root.minsize(980, 780)

    top = ttk.Label(root, font=("SF Pro Text", 13, "bold"),
                    text="правая рука: X — база, Y — плечо   |   "
                         "левая рука: Y — локоть")
    top.pack(pady=(12, 6))

    video = tk.Label(root, background="#111", foreground="#777",
                     text="ожидание кадра...", font=("SF Pro Text", 12))
    video.pack(padx=16)

    table = ttk.Frame(root)
    table.pack(fill="x", padx=16, pady=12)

    headers = ("ось", "id", "цель", "факт", "ошибка", "об/мин", "состояние", "рука")
    for column, title in enumerate(headers):
        ttk.Label(table, text=title, font=("SF Pro Text", 11, "bold")).grid(
            row=0, column=column, padx=8, sticky="w")
    table.grid_columnconfigure(7, weight=1)

    rows = {}
    for index, spec in enumerate(specs, start=1):
        labels = []
        for column in range(len(headers)):
            label = ttk.Label(table, text="—", font=("SF Pro Text", 12))
            label.grid(row=index, column=column, padx=8, sticky="w")
            labels.append(label)
        labels[0].config(text=spec.name)
        labels[1].config(text=str(spec.device_id))
        labels[7].config(text=f"{spec.hand} · {spec.source.upper()}")
        rows[spec.name] = labels

    status = ttk.Label(root, text="", font=("SF Pro Text", 11))
    status.pack(pady=(0, 6))

    buttons = ttk.Frame(root)
    buttons.pack(pady=(0, 14))

    def toggle_pause():
        paused["value"] = not paused["value"]
        pause_button.config(text="ПРОДОЛЖИТЬ" if paused["value"] else "ПАУЗА")

    def emergency_stop():
        paused["value"] = True
        loop.stop()
        pause_button.config(text="ПРОДОЛЖИТЬ")
        status.config(text="ОСТАНОВЛЕНО кнопкой. Перезапустите приложение.",
                      foreground="#e05252")

    pause_button = ttk.Button(buttons, text="ПАУЗА", command=toggle_pause)
    pause_button.grid(row=0, column=0, padx=6)
    ttk.Button(buttons, text="СТОП", command=emergency_stop).grid(
        row=0, column=1, padx=6)
    ttk.Button(buttons, text="ноль здесь",
               command=loop.set_zero_here).grid(row=0, column=2, padx=6)

    def poll_camera():
        hands = source.poll()
        if not paused["value"]:
            targets = {}
            for spec in specs:
                wrist = hands.get(spec.hand)
                if wrist is None:
                    continue        # рука не в кадре — цель этой оси замирает
                x, y = wrist
                targets[spec.name] = spec.target_deg({"x": x, "y": y}[spec.source])
            if targets:
                loop.set_targets(targets)
        root.after(30, poll_camera)

    def draw():
        frame = source.last_frame
        if frame is not None:
            image = Image.fromarray(frame)
            image.thumbnail((720, 420))
            photo = ImageTk.PhotoImage(image)
            photo_holder["frame"] = photo
            video.configure(image=photo, text="")

        for spec in specs:
            follower = loop.followers[spec.name]
            t = follower.telemetry
            labels = rows[spec.name]
            labels[2].config(text=f"{t.target_deg:7.1f}°")
            labels[3].config(text=f"{t.actual_deg:7.1f}°")
            labels[4].config(text=f"{t.error_deg:+7.1f}°")
            labels[5].config(text=f"{t.command_rpm:+3d}")
            present = spec.hand in source.hands
            if not present:
                labels[6].config(text="руки нет — замерла")
            else:
                labels[6].config(text="удержание" if t.holding else "едет")

        if loop.error_text:
            status.config(text=f"ОСТАНОВЛЕНО: {loop.error_text}",
                          foreground="#e05252")
        elif loop.warning_text:
            status.config(text=loop.warning_text, foreground="#ffb648")
        else:
            seen = sorted(source.hands)
            hand = ("в кадре: " + ", ".join(seen)) if seen else "РУК НЕТ — цели замерли"
            mode = "ПАУЗА" if paused["value"] else hand
            drops = f"   сбоев {loop.dropped}" if loop.dropped else ""
            status.config(
                text=f"{mode}   обход {loop.sweep_ms:.0f} мс   "
                     f"всего {loop.sweeps}{drops}",
                foreground="",
            )
        root.after(50, draw)

    def shutdown():
        loop.stop()
        source.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", shutdown)
    loop.start()
    poll_camera()
    draw()
    try:
        root.mainloop()
    finally:
        loop.stop()
        source.close()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--simulate", action="store_true", help="приводы на бумаге")
    p.add_argument("--confirm", action="store_true",
                   help="подтвердить, что оси разгружены (нужно без --simulate)")

    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=19200)
    p.add_argument("--parity", default="E", choices=["N", "E", "O"])
    p.add_argument("--host", help="Modbus TCP через шлюз")
    p.add_argument("--tcp-port", type=int, default=502)

    p.add_argument("--cpr", type=int, default=131072)
    p.add_argument("--no-invert", action="store_true")
    p.add_argument("--max-rpm", type=int, default=15)
    p.add_argument("--kp", type=float, default=0.225)
    p.add_argument("--tolerance", type=float, default=2.5)
    p.add_argument("--period", type=float, default=0.12,
                   help="период контура одной оси, с")
    p.add_argument("--sweep", type=float, default=0.12,
                   help="период обхода всех осей, с")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--skip-line-check", action="store_true",
                   help="не проверять шину на шум перед запуском")
    p.add_argument("--swap-hands", action="store_true",
                   help="поменять руки местами, если метки вышли наоборот")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(message)s")

    if not args.simulate and not args.confirm:
        print(__doc__)
        print("Запустите с --confirm, когда оси будут разгружены, "
              "или с --simulate для проверки без железа.")
        return 1

    specs = load_axes()
    print("оси:")
    for spec in specs:
        print(f"  id={spec.device_id}  {spec.name:<10} <- {spec.hand} {spec.source.upper()}, "
              f"ход ±{spec.span_deg/2:.0f}°")
    print()

    followers, bus = build_followers(args, specs)
    loop = MultiAxisFollower(followers, period_s=args.sweep)

    for spec in specs:
        log.info("опрашиваю ось %s (id=%d)...", spec.name, spec.device_id)
        try:
            followers[spec.name].set_zero_here()
        except Exception as exc:
            if bus is not None:
                bus.close()
            raise SystemExit(
                f"ось {spec.name} (id={spec.device_id}) не отвечает: {exc}\n"
                f"проверьте питание привода и его адрес:\n"
                f"    python examples/scan_bus.py {getattr(args, 'port', None) or ''} "
                f"--baud {args.baud} --parity {args.parity} --ids 1-8"
            ) from exc
    log.info("положение всех осей принято за ноль")

    source = HandSource(camera_index=args.camera, swap_hands=args.swap_hands)
    try:
        return run_gui(loop, source, specs, args)
    finally:
        if bus is not None:
            try:
                bus.stop_all()
            finally:
                bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
