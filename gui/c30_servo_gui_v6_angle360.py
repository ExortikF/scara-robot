# -*- coding: utf-8 -*-
"""
C30 Servo Control v5
JOG + circular 0..360° position PID + conservative PID auto-tuning over Modbus RTU.

IMPORTANT:
This program is a supervisory Windows/USB-RS485 loop. It is NOT a hard
real-time motion controller. Keep independent hardware E-STOP and limits.

C30 registers used:
P-076  0x004C : JOG speed [r/min]
P-098  0x0062 : forced Servo Enable
P-181  0x00B5 : Modbus ID
0x1000        : actual motor speed [r/min]
0x1010        : JOG command
                0x1234 = Servo enabled / zero speed hold
                0x2222 = forward
                0x1111 = reverse
                0x0000 = exit JOG / stop
0x101A        : alarm code
0x101F        : absolute position LOW
0x1020        : absolute position HIGH
"""

import json
import queue
import statistics
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

import serial
import sys
from serial.tools import list_ports


IS_MAC = sys.platform == "darwin"

# "Segoe UI" на macOS нет — Tk молча подставит замену и разъедет вёрстку.
UI_FONT = "SF Pro Text" if IS_MAC else "Segoe UI"

# Порты, которые на macOS показывать нельзя:
#   /dev/tty.*  — dial-in, open() виснет в ожидании DCD;
#   Bluetooth и debug-консоли — к RS-485 отношения не имеют.
MAC_PORT_JUNK = ("Bluetooth", "wlan-debug", "debug-console")

# VID чипов USB<->RS-485: FTDI, WCH (CH340), Silicon Labs, Prolific.
USB_SERIAL_VIDS = (0x0403, 0x1A86, 0x10C4, 0x067B)


def usable_ports():
    """Порты, которые имеет смысл предлагать пользователю."""
    ports = list(list_ports.comports())
    if IS_MAC:
        ports = [
            p for p in ports
            if not p.device.startswith("/dev/tty.")
            and not any(junk in p.device for junk in MAC_PORT_JUNK)
        ]
    ports.sort(key=lambda p: (p.vid not in USB_SERIAL_VIDS, p.device))
    return [p.device for p in ports]


APP_TITLE = "C30 Servo Control v6 — 0…360° + PID Autotune"
SETTINGS_FILE = Path(__file__).with_name("c30_servo_settings.json")

REG_P076_JOG_SPEED = 0x004C
REG_P098_SERVO_ENABLE = 0x0062
REG_P181_COMM_ID = 0x00B5
REG_ACTUAL_SPEED = 0x1000
REG_JOG_COMMAND = 0x1010
REG_ALARM = 0x101A
REG_ABS_POS_LOW = 0x101F

CMD_STOP = 0x0000
CMD_HOLD = 0x1234
CMD_FORWARD = 0x2222
CMD_REVERSE = 0x1111

BAUD_MAP = {
    0: 4800,
    1: 9600,
    2: 19200,
    3: 38400,
    4: 57600,
    5: 115200,
}

MODE_MAP = {
    0: "8N1",
    1: "8E1",
    2: "8O1",
    3: "8N2",
    4: "8E2",
    5: "8O2",
}


class ModbusError(Exception):
    pass


class ModbusRTUClient:
    def __init__(self):
        self.ser = None
        self.slave_id = 1
        self._lock = threading.Lock()
        self.inter_frame_delay = 0.03
        self.retries = 2
        self._last_transaction_end = 0.0

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    @classmethod
    def add_crc(cls, frame: bytes) -> bytes:
        crc = cls.crc16(frame)
        return frame + bytes((crc & 0xFF, (crc >> 8) & 0xFF))

    @classmethod
    def valid_crc(cls, frame: bytes) -> bool:
        if len(frame) < 4:
            return False
        expected = cls.crc16(frame[:-2])
        received = frame[-2] | (frame[-1] << 8)
        return expected == received

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, slave_id, baudrate, parity, stopbits, timeout):
        self.close()
        parity_value = {
            "N": serial.PARITY_NONE,
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD,
        }[parity]

        self.ser = serial.Serial(
            port=port,
            baudrate=int(baudrate),
            bytesize=serial.EIGHTBITS,
            parity=parity_value,
            stopbits=int(stopbits),
            timeout=float(timeout),
            write_timeout=float(timeout),
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.slave_id = int(slave_id)
        time.sleep(0.10)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            finally:
                self.ser = None

    def _ensure_open(self):
        if not self.is_open:
            raise ModbusError("COM-порт не открыт.")

    def _read_exact(self, count):
        data = bytearray()
        deadline = time.monotonic() + max(float(self.ser.timeout or 0.5) * 1.5, 0.2)
        while len(data) < count and time.monotonic() < deadline:
            chunk = self.ser.read(count - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def _exchange(self, pdu: bytes, expected_fc: int) -> bytes:
        self._ensure_open()
        last_error = None

        with self._lock:
            for attempt in range(self.retries + 1):
                try:
                    elapsed = time.monotonic() - self._last_transaction_end
                    if elapsed < self.inter_frame_delay:
                        time.sleep(self.inter_frame_delay - elapsed)

                    self.ser.reset_input_buffer()
                    request = self.add_crc(bytes([self.slave_id]) + pdu)
                    self.ser.write(request)
                    self.ser.flush()

                    head = self._read_exact(3)
                    if len(head) != 3:
                        raise ModbusError("Нет ответа от привода (timeout).")

                    addr, fc, third = head

                    if addr != self.slave_id:
                        raise ModbusError(
                            f"Ответ Slave={addr}, ожидался Slave={self.slave_id}."
                        )

                    if fc == (expected_fc | 0x80):
                        tail = self._read_exact(2)
                        frame = head + tail
                        if len(frame) != 5 or not self.valid_crc(frame):
                            raise ModbusError("Ошибка CRC в exception-ответе.")
                        raise RuntimeError(
                            f"Modbus exception 0x{third:02X}, FC=0x{expected_fc:02X}."
                        )

                    if fc != expected_fc:
                        raise ModbusError(
                            f"Ответ FC=0x{fc:02X}, ожидался FC=0x{expected_fc:02X}."
                        )

                    if fc == 0x03:
                        byte_count = third
                        tail = self._read_exact(byte_count + 2)
                        frame = head + tail
                        if len(frame) != 3 + byte_count + 2:
                            raise ModbusError("Короткий ответ FC03.")
                    elif fc == 0x06:
                        tail = self._read_exact(5)
                        frame = head + tail
                        if len(frame) != 8:
                            raise ModbusError("Короткий ответ FC06.")
                    else:
                        raise ModbusError(f"FC 0x{fc:02X} не поддержан приложением.")

                    if not self.valid_crc(frame):
                        raise ModbusError("CRC ответа неверен.")

                    self._last_transaction_end = time.monotonic()
                    return frame

                except RuntimeError:
                    self._last_transaction_end = time.monotonic()
                    raise
                except (ModbusError, serial.SerialException, OSError) as exc:
                    last_error = exc
                    self._last_transaction_end = time.monotonic()
                    if attempt < self.retries:
                        try:
                            self.ser.reset_input_buffer()
                        except Exception:
                            pass
                        time.sleep(0.05)
                        continue

                    raise ModbusError(
                        f"{last_error} (после {self.retries + 1} попыток)"
                    ) from last_error

    def read_holding(self, address, count=1):
        if not 1 <= count <= 125:
            raise ValueError("count должен быть 1..125")

        pdu = bytes((
            0x03,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        ))
        frame = self._exchange(pdu, 0x03)
        byte_count = frame[2]

        if byte_count != count * 2:
            raise ModbusError("Неверное число байт в FC03.")

        data = frame[3:3 + byte_count]
        values = []
        for i in range(0, len(data), 2):
            values.append((data[i] << 8) | data[i + 1])
        return values

    def write_single(self, address, value):
        if not 0 <= value <= 0xFFFF:
            raise ValueError("Значение регистра должно быть 0..65535.")

        pdu = bytes((
            0x06,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ))
        frame = self._exchange(pdu, 0x06)

        response_address = (frame[2] << 8) | frame[3]
        response_value = (frame[4] << 8) | frame[5]

        if response_address != address or response_value != value:
            raise ModbusError("FC06 подтвердил другой адрес/значение.")


def parse_float(value):
    return float(str(value).strip().replace(",", "."))


def u16_to_i16(value):
    return value - 0x10000 if value & 0x8000 else value


def words_to_i32(low_word, high_word):
    value = ((high_word & 0xFFFF) << 16) | (low_word & 0xFFFF)
    return value - 0x100000000 if value & 0x80000000 else value


def i32_delta(current, start):
    """Signed 32-bit difference with wrap handling."""
    delta = (int(current) - int(start)) & 0xFFFFFFFF
    return delta - 0x100000000 if delta & 0x80000000 else delta


def normalize_angle_360(angle_deg):
    """Normalize any angle to the range 0 <= angle < 360."""
    angle = float(angle_deg) % 360.0
    # Avoid displaying 360.000 because of floating point rounding.
    if angle >= 359.999999999:
        return 0.0
    return angle


def shortest_angle_error(target_deg, actual_deg):
    """Shortest signed error in degrees, range [-180, 180)."""
    return ((float(target_deg) - float(actual_deg) + 180.0) % 360.0) - 180.0


class C30ServoApp:
    def __init__(self, root):
        self.root = root
        self.root.title(APP_TITLE)
        # 930 px не влезают в рабочую область MacBook: окно уезжает под Dock
        # и нижние кнопки становятся недоступны.
        screen_height = self.root.winfo_screenheight()
        self.root.minsize(1000, min(930, max(600, screen_height - 160)))

        self.client = ModbusRTUClient()
        self.ui_queue = queue.Queue()

        self.polling = False
        self.poll_in_flight = False
        self.latest_abs_raw = None
        self.zero_offset_counts = 0

        self.pid_active = False
        self.pid_stop_event = threading.Event()
        self.pid_thread = None

        self.autotune_active = False
        self.autotune_stop_event = threading.Event()
        self.autotune_thread = None

        self._build_ui()
        self._load_settings()
        self.refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self._process_ui_queue)
        self.root.after(500, self._poll_tick)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(4, weight=1)

        comm = ttk.LabelFrame(self.root, text="Modbus RTU / COM")
        comm.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))

        ttk.Label(comm, text="COM:").grid(row=0, column=0, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            comm, textvariable=self.port_var, width=12, state="readonly"
        )
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(comm, text="Обновить", command=self.refresh_ports).grid(
            row=0, column=2, padx=5, pady=5
        )

        ttk.Label(comm, text="Slave ID:").grid(row=0, column=3, padx=5, pady=5)
        self.slave_var = tk.StringVar(value="1")
        ttk.Spinbox(comm, from_=1, to=32, textvariable=self.slave_var, width=6).grid(
            row=0, column=4, padx=5, pady=5
        )

        ttk.Label(comm, text="Baud:").grid(row=0, column=5, padx=5, pady=5)
        self.baud_var = tk.StringVar(value="19200")
        ttk.Combobox(
            comm,
            textvariable=self.baud_var,
            values=("4800", "9600", "19200", "38400", "57600", "115200"),
            width=10,
            state="readonly",
        ).grid(row=0, column=6, padx=5, pady=5)

        ttk.Label(comm, text="Parity:").grid(row=0, column=7, padx=5, pady=5)
        self.parity_var = tk.StringVar(value="E")
        ttk.Combobox(
            comm, textvariable=self.parity_var,
            values=("N", "E", "O"), width=6, state="readonly"
        ).grid(row=0, column=8, padx=5, pady=5)

        ttk.Label(comm, text="Stop:").grid(row=0, column=9, padx=5, pady=5)
        self.stopbits_var = tk.StringVar(value="1")
        ttk.Combobox(
            comm, textvariable=self.stopbits_var,
            values=("1", "2"), width=5, state="readonly"
        ).grid(row=0, column=10, padx=5, pady=5)

        ttk.Label(comm, text="Timeout, s:").grid(row=0, column=11, padx=5, pady=5)
        self.timeout_var = tk.StringVar(value="0.50")
        ttk.Entry(comm, textvariable=self.timeout_var, width=7).grid(
            row=0, column=12, padx=5, pady=5
        )

        ttk.Button(comm, text="Подключиться", command=self.connect).grid(
            row=1, column=0, columnspan=3, padx=5, pady=5, sticky="ew"
        )
        ttk.Button(comm, text="Проверить связь", command=self.test_connection).grid(
            row=1, column=3, columnspan=3, padx=5, pady=5, sticky="ew"
        )
        ttk.Button(comm, text="Отключиться", command=self.disconnect).grid(
            row=1, column=6, columnspan=3, padx=5, pady=5, sticky="ew"
        )

        self.comm_status_var = tk.StringVar(value="Не подключено")
        ttk.Label(comm, textvariable=self.comm_status_var).grid(
            row=2, column=0, columnspan=13, padx=8, pady=(2, 7), sticky="w"
        )

        status = ttk.LabelFrame(self.root, text="Состояние привода")
        status.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        for i in range(8):
            status.columnconfigure(i, weight=1)

        ttk.Label(status, text="Фактическая скорость:").grid(
            row=0, column=0, padx=6, pady=7, sticky="e"
        )
        self.actual_speed_var = tk.StringVar(value="—")
        ttk.Label(
            status, textvariable=self.actual_speed_var,
            font=(UI_FONT, 13, "bold")
        ).grid(row=0, column=1, padx=6, pady=7, sticky="w")

        ttk.Label(status, text="Направление:").grid(
            row=0, column=2, padx=6, pady=7, sticky="e"
        )
        self.actual_dir_var = tk.StringVar(value="—")
        ttk.Label(
            status, textvariable=self.actual_dir_var,
            font=(UI_FONT, 11, "bold")
        ).grid(row=0, column=3, padx=6, pady=7, sticky="w")

        ttk.Label(status, text="Alarm:").grid(
            row=0, column=4, padx=6, pady=7, sticky="e"
        )
        self.alarm_var = tk.StringVar(value="—")
        ttk.Label(
            status, textvariable=self.alarm_var,
            font=(UI_FONT, 11, "bold")
        ).grid(row=0, column=5, padx=6, pady=7, sticky="w")

        ttk.Label(status, text="Абс. encoder RAW:").grid(
            row=1, column=0, padx=6, pady=7, sticky="e"
        )
        self.abs_position_var = tk.StringVar(value="—")
        ttk.Label(
            status, textvariable=self.abs_position_var,
            font=(UI_FONT, 12, "bold")
        ).grid(row=1, column=1, padx=6, pady=7, sticky="w")

        self.abs_words_var = tk.StringVar(value="LOW=—  HIGH=—")
        ttk.Label(status, textvariable=self.abs_words_var).grid(
            row=1, column=2, columnspan=2, padx=6, pady=7, sticky="w"
        )

        ttk.Label(status, text="Угол 0…360°:").grid(
            row=1, column=4, padx=6, pady=7, sticky="e"
        )
        self.angle_var = tk.StringVar(value="—")
        ttk.Label(
            status, textvariable=self.angle_var,
            font=(UI_FONT, 13, "bold")
        ).grid(row=1, column=5, padx=6, pady=7, sticky="w")

        ttk.Button(
            status, text="Текущую позицию = 0°",
            command=self.set_software_zero
        ).grid(row=1, column=6, columnspan=2, padx=6, pady=7, sticky="ew")

        ttk.Label(status, text="Обороты от программного 0:").grid(
            row=2, column=0, padx=6, pady=(0, 7), sticky="e"
        )
        self.turns_var = tk.StringVar(value="—")
        ttk.Label(status, textvariable=self.turns_var).grid(
            row=2, column=1, columnspan=2, padx=6, pady=(0, 7), sticky="w"
        )

        jog = ttk.LabelFrame(self.root, text="Ручное управление JOG")
        jog.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        for i in range(7):
            jog.columnconfigure(i, weight=1)

        ttk.Label(jog, text="Скорость, rpm:").grid(
            row=0, column=0, padx=6, pady=7, sticky="e"
        )
        self.speed_var = tk.StringVar(value="100")
        ttk.Spinbox(
            jog, from_=0, to=32000, increment=10,
            textvariable=self.speed_var, width=10
        ).grid(row=0, column=1, padx=6, pady=7)

        ttk.Label(jog, text="Направление:").grid(
            row=0, column=2, padx=6, pady=7, sticky="e"
        )
        self.direction_var = tk.StringVar(value="forward")
        ttk.Radiobutton(
            jog, text="Вперед / CCW",
            variable=self.direction_var, value="forward"
        ).grid(row=0, column=3, padx=4, pady=7)
        ttk.Radiobutton(
            jog, text="Назад / CW",
            variable=self.direction_var, value="reverse"
        ).grid(row=0, column=4, padx=4, pady=7)

        ttk.Button(jog, text="SERVO ON", command=self.servo_on).grid(
            row=1, column=0, padx=6, pady=7, sticky="ew"
        )
        ttk.Button(jog, text="SERVO OFF", command=self.servo_off).grid(
            row=1, column=1, padx=6, pady=7, sticky="ew"
        )
        ttk.Button(jog, text="Применить скорость", command=self.apply_speed).grid(
            row=1, column=2, padx=6, pady=7, sticky="ew"
        )
        ttk.Button(jog, text="▶ ПУСК", command=self.start_motion).grid(
            row=1, column=3, columnspan=2, padx=6, pady=7, sticky="ew"
        )
        ttk.Button(jog, text="■ СТОП", command=self.stop_motion).grid(
            row=1, column=5, columnspan=2, padx=6, pady=7, sticky="ew"
        )

        pid = ttk.LabelFrame(
            self.root,
            text="Положение по PID + автонастройка"
        )
        pid.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
        for i in range(10):
            pid.columnconfigure(i, weight=1)

        ttk.Label(pid, text="Counts/оборот:").grid(
            row=0, column=0, padx=5, pady=5, sticky="e"
        )
        self.cpr_var = tk.StringVar(value="131072")
        ttk.Entry(pid, textvariable=self.cpr_var, width=10).grid(
            row=0, column=1, padx=5, pady=5
        )

        self.invert_encoder_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            pid, text="Инверсия знака encoder",
            variable=self.invert_encoder_var
        ).grid(row=0, column=2, columnspan=2, padx=5, pady=5, sticky="w")

        ttk.Label(pid, text="Цель 0…360°, °:").grid(
            row=0, column=4, padx=5, pady=5, sticky="e"
        )
        self.pid_target_var = tk.StringVar(value="90")
        ttk.Entry(pid, textvariable=self.pid_target_var, width=10).grid(
            row=0, column=5, padx=5, pady=5
        )

        self.pid_status_var = tk.StringVar(value="PID остановлен")
        ttk.Label(
            pid, textvariable=self.pid_status_var,
            font=(UI_FONT, 10, "bold")
        ).grid(row=0, column=6, columnspan=4, padx=5, pady=5, sticky="w")

        ttk.Label(pid, text="Kp:").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.kp_var = tk.StringVar(value="0.20")
        ttk.Entry(pid, textvariable=self.kp_var, width=9).grid(
            row=1, column=1, padx=5, pady=5
        )

        ttk.Label(pid, text="Ki:").grid(row=1, column=2, padx=5, pady=5, sticky="e")
        self.ki_var = tk.StringVar(value="0.00")
        ttk.Entry(pid, textvariable=self.ki_var, width=9).grid(
            row=1, column=3, padx=5, pady=5
        )

        ttk.Label(pid, text="Kd:").grid(row=1, column=4, padx=5, pady=5, sticky="e")
        self.kd_var = tk.StringVar(value="0.05")
        ttk.Entry(pid, textvariable=self.kd_var, width=9).grid(
            row=1, column=5, padx=5, pady=5
        )

        ttk.Label(pid, text="Допуск, °:").grid(
            row=1, column=6, padx=5, pady=5, sticky="e"
        )
        self.pid_tol_var = tk.StringVar(value="0.5")
        ttk.Entry(pid, textvariable=self.pid_tol_var, width=9).grid(
            row=1, column=7, padx=5, pady=5
        )

        ttk.Label(pid, text="Min rpm:").grid(
            row=2, column=0, padx=5, pady=5, sticky="e"
        )
        self.pid_min_rpm_var = tk.StringVar(value="1")
        ttk.Entry(pid, textvariable=self.pid_min_rpm_var, width=9).grid(
            row=2, column=1, padx=5, pady=5
        )

        ttk.Label(pid, text="Max rpm:").grid(
            row=2, column=2, padx=5, pady=5, sticky="e"
        )
        self.pid_max_rpm_var = tk.StringVar(value="50")
        ttk.Entry(pid, textvariable=self.pid_max_rpm_var, width=9).grid(
            row=2, column=3, padx=5, pady=5
        )

        self.pid_error_var = tk.StringVar(value="Ошибка: —")
        ttk.Label(pid, textvariable=self.pid_error_var).grid(
            row=2, column=4, padx=5, pady=5, sticky="w"
        )
        self.pid_output_var = tk.StringVar(value="Выход: — rpm")
        ttk.Label(pid, textvariable=self.pid_output_var).grid(
            row=2, column=5, padx=5, pady=5, sticky="w"
        )

        ttk.Button(pid, text="▶ START PID", command=self.start_pid).grid(
            row=2, column=6, columnspan=2, padx=5, pady=5, sticky="ew"
        )
        ttk.Button(pid, text="■ STOP PID", command=self.stop_pid).grid(
            row=2, column=8, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        sep = ttk.Separator(pid, orient="horizontal")
        sep.grid(row=3, column=0, columnspan=10, sticky="ew", padx=5, pady=7)

        ttk.Label(pid, text="Автонастройка: тестовая скорость, rpm").grid(
            row=4, column=0, columnspan=2, padx=5, pady=5, sticky="e"
        )
        self.tune_rpm_var = tk.StringVar(value="10")
        ttk.Entry(pid, textvariable=self.tune_rpm_var, width=9).grid(
            row=4, column=2, padx=5, pady=5
        )

        ttk.Label(pid, text="Ход теста, °:").grid(
            row=4, column=3, padx=5, pady=5, sticky="e"
        )
        self.tune_angle_var = tk.StringVar(value="15")
        ttk.Entry(pid, textvariable=self.tune_angle_var, width=9).grid(
            row=4, column=4, padx=5, pady=5
        )

        ttk.Label(pid, text="Профиль:").grid(
            row=4, column=5, padx=5, pady=5, sticky="e"
        )
        self.tune_profile_var = tk.StringVar(value="Осторожный")
        ttk.Combobox(
            pid,
            textvariable=self.tune_profile_var,
            values=("Осторожный", "Нормальный", "Быстрый"),
            width=13,
            state="readonly",
        ).grid(row=4, column=6, padx=5, pady=5)

        ttk.Button(
            pid, text="⚙ АВТОНАСТРОЙКА PID",
            command=self.start_autotune
        ).grid(row=4, column=7, columnspan=2, padx=5, pady=5, sticky="ew")

        ttk.Button(
            pid, text="СТОП НАСТРОЙКИ",
            command=self.stop_autotune
        ).grid(row=4, column=9, padx=5, pady=5, sticky="ew")

        self.autotune_status_var = tk.StringVar(
            value="Автонастройка не запускалась"
        )
        ttk.Label(
            pid, textvariable=self.autotune_status_var,
            wraplength=900
        ).grid(
            row=5, column=0, columnspan=10,
            padx=7, pady=(3, 7), sticky="w"
        )

        warning = (
            "Автонастройка сама кратковременно вращает двигатель вперед, измеряет "
            "скоростной отклик, затем возвращает вал к исходному углу и рассчитывает "
            "Kp/Ki/Kd. Первый тест: без нагрузки, 10 rpm, 15°, профиль «Осторожный»."
        )
        ttk.Label(pid, text=warning, wraplength=930).grid(
            row=6, column=0, columnspan=10,
            padx=7, pady=(0, 8), sticky="w"
        )

        log_frame = ttk.LabelFrame(self.root, text="Журнал")
        log_frame.grid(
            row=4, column=0, sticky="nsew",
            padx=10, pady=(5, 10)
        )
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        self.log_text = tk.Text(
            log_frame, height=12, wrap="word", state="disabled"
        )
        self.log_text.grid(row=0, column=0, sticky="nsew", padx=(5, 0), pady=5)

        sb = ttk.Scrollbar(
            log_frame, orient="vertical",
            command=self.log_text.yview
        )
        sb.grid(row=0, column=1, sticky="ns", pady=5)
        self.log_text.configure(yscrollcommand=sb.set)

    def log(self, text):
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {text}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # ----------------------------------------------------------- Settings
    def refresh_ports(self):
        ports = usable_ports()
        current = self.port_var.get()
        self.port_combo["values"] = ports
        if current in ports:
            self.port_var.set(current)
        elif not IS_MAC and "COM4" in ports:
            self.port_var.set("COM4")
        elif ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def _load_settings(self):
        if not SETTINGS_FILE.exists():
            return
        try:
            data = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
            self.port_var.set(str(data.get("port", self.port_var.get())))
            self.slave_var.set(str(data.get("slave_id", 1)))
            self.baud_var.set(str(data.get("baudrate", 19200)))
            self.parity_var.set(str(data.get("parity", "E")))
            self.stopbits_var.set(str(data.get("stopbits", 1)))
            self.timeout_var.set(str(data.get("timeout", 0.5)))

            self.speed_var.set(str(data.get("speed_rpm", 100)))
            self.direction_var.set(str(data.get("direction", "forward")))

            self.zero_offset_counts = int(data.get("zero_offset_counts", 0))
            self.cpr_var.set(str(data.get("counts_per_rev", 131072)))
            self.invert_encoder_var.set(bool(data.get("invert_encoder", False)))

            self.pid_target_var.set(str(data.get("pid_target_deg", 90)))
            self.kp_var.set(str(data.get("pid_kp", 0.20)))
            self.ki_var.set(str(data.get("pid_ki", 0.00)))
            self.kd_var.set(str(data.get("pid_kd", 0.05)))
            self.pid_tol_var.set(str(data.get("pid_tolerance_deg", 0.5)))
            self.pid_min_rpm_var.set(str(data.get("pid_min_rpm", 1)))
            self.pid_max_rpm_var.set(str(data.get("pid_max_rpm", 50)))

            self.tune_rpm_var.set(str(data.get("tune_rpm", 10)))
            self.tune_angle_var.set(str(data.get("tune_angle", 15)))
            self.tune_profile_var.set(
                str(data.get("tune_profile", "Осторожный"))
            )
        except Exception:
            pass

    def _save_settings(self):
        data = {
            "port": self.port_var.get(),
            "slave_id": self.slave_var.get(),
            "baudrate": self.baud_var.get(),
            "parity": self.parity_var.get(),
            "stopbits": self.stopbits_var.get(),
            "timeout": self.timeout_var.get(),
            "speed_rpm": self.speed_var.get(),
            "direction": self.direction_var.get(),

            "zero_offset_counts": int(self.zero_offset_counts),
            "counts_per_rev": self.cpr_var.get(),
            "invert_encoder": bool(self.invert_encoder_var.get()),

            "pid_target_deg": self.pid_target_var.get(),
            "pid_kp": self.kp_var.get(),
            "pid_ki": self.ki_var.get(),
            "pid_kd": self.kd_var.get(),
            "pid_tolerance_deg": self.pid_tol_var.get(),
            "pid_min_rpm": self.pid_min_rpm_var.get(),
            "pid_max_rpm": self.pid_max_rpm_var.get(),

            "tune_rpm": self.tune_rpm_var.get(),
            "tune_angle": self.tune_angle_var.get(),
            "tune_profile": self.tune_profile_var.get(),
        }
        try:
            SETTINGS_FILE.write_text(
                json.dumps(data, ensure_ascii=False, indent=2),
                encoding="utf-8"
            )
        except Exception:
            pass

    # ------------------------------------------------------------- Helpers
    def _run_async(self, func, success=None, failure=None):
        def worker():
            try:
                result = func()
                self.ui_queue.put(("success", success, result))
            except Exception as exc:
                self.ui_queue.put(("failure", failure, exc))

        threading.Thread(target=worker, daemon=True).start()

    def _process_ui_queue(self):
        try:
            while True:
                _, callback, payload = self.ui_queue.get_nowait()
                if callback:
                    callback(payload)
        except queue.Empty:
            pass
        self.root.after(100, self._process_ui_queue)

    def _require_connection(self):
        if not self.client.is_open:
            raise ModbusError("Сначала подключитесь к приводу.")

    def _get_comm_settings(self):
        port = self.port_var.get().strip()
        if not port:
            raise ValueError("Выберите COM-порт.")

        slave = int(self.slave_var.get())
        if not 1 <= slave <= 32:
            raise ValueError("Slave ID должен быть 1..32.")

        baud = int(self.baud_var.get())
        parity = self.parity_var.get()
        stopbits = int(self.stopbits_var.get())
        timeout = parse_float(self.timeout_var.get())
        if timeout <= 0:
            raise ValueError("Timeout должен быть > 0.")

        return port, slave, baud, parity, stopbits, timeout

    def _get_speed(self):
        speed = int(round(parse_float(self.speed_var.get())))
        if not 0 <= speed <= 32000:
            raise ValueError("Скорость должна быть 0..32000 rpm.")
        return speed

    def _get_cpr(self):
        cpr = parse_float(self.cpr_var.get())
        if cpr <= 0:
            raise ValueError("Counts/оборот должен быть > 0.")
        return cpr

    def relative_counts(self, raw):
        delta = i32_delta(raw, self.zero_offset_counts)
        if self.invert_encoder_var.get():
            delta = -delta
        return delta

    def counts_to_degrees(self, raw):
        """
        Mechanical angle within ONE revolution.
        Always returns 0 <= angle < 360 deg.
        Multi-turn accumulation remains available separately as turns.
        """
        delta = self.relative_counts(raw)
        cpr = self._get_cpr()
        one_turn_counts = delta % cpr
        return normalize_angle_360(one_turn_counts * 360.0 / cpr)

    def counts_to_turns(self, raw):
        """Accumulated mechanical revolutions relative to software zero."""
        return self.relative_counts(raw) / self._get_cpr()

    def delta_counts_to_degrees(self, current, start):
        """
        Unwrapped signed angular travel. Used only by autotune,
        where the test move is intentionally less than one revolution.
        """
        delta = i32_delta(current, start)
        if self.invert_encoder_var.get():
            delta = -delta
        return delta * 360.0 / self._get_cpr()

    def read_absolute_raw(self):
        regs = self.client.read_holding(REG_ABS_POS_LOW, 2)
        return words_to_i32(regs[0], regs[1]), regs[0], regs[1]

    def set_software_zero(self):
        if self.latest_abs_raw is None:
            messagebox.showwarning(
                "Ноль",
                "Нет данных энкодера. Сначала подключитесь к приводу."
            )
            return

        self.zero_offset_counts = int(self.latest_abs_raw)
        self.angle_var.set("0.000°")
        self.turns_var.set("0.0000")
        self._save_settings()
        self.log(
            f"Установлен программный 0°: offset RAW={self.zero_offset_counts}. "
            "RAW абсолютного энкодера при этом не меняется."
        )

    # ----------------------------------------------------------- Connection
    def connect(self):
        try:
            settings = self._get_comm_settings()
        except Exception as exc:
            messagebox.showerror("Настройки", str(exc))
            return

        self.comm_status_var.set("Подключение...")

        def job():
            self.client.open(*settings)
            return self.client.read_holding(REG_P181_COMM_ID, 3)

        def ok(regs):
            self.polling = True
            self._save_settings()
            p181, p182, p183 = regs
            self.comm_status_var.set(
                f"Связь OK | P181={p181} | "
                f"P182={p182} ({BAUD_MAP.get(p182, '?')}) | "
                f"P183={p183} ({MODE_MAP.get(p183, '?')})"
            )
            self.log("Modbus RTU подключен.")

        def fail(exc):
            self.polling = False
            self.client.close()
            self.comm_status_var.set("Ошибка подключения")
            self.log(f"Ошибка подключения: {exc}")
            messagebox.showerror("Modbus RTU", str(exc))

        self._run_async(job, ok, fail)

    def test_connection(self):
        if not self.client.is_open:
            self.connect()
            return

        def job():
            return self.client.read_holding(REG_P181_COMM_ID, 3)

        def ok(regs):
            p181, p182, p183 = regs
            self.comm_status_var.set(
                f"Связь OK | P181={p181} | "
                f"P182={p182} ({BAUD_MAP.get(p182, '?')}) | "
                f"P183={p183} ({MODE_MAP.get(p183, '?')})"
            )
            self.log("Проверка связи: OK.")

        def fail(exc):
            self.log(f"Проверка связи: {exc}")
            messagebox.showerror("Проверка связи", str(exc))

        self._run_async(job, ok, fail)

    def disconnect(self):
        if self.pid_active:
            self.stop_pid()
        if self.autotune_active:
            self.stop_autotune()

        self.polling = False
        self.client.close()
        self.comm_status_var.set("Не подключено")
        self.actual_speed_var.set("—")
        self.actual_dir_var.set("—")
        self.alarm_var.set("—")
        self.abs_position_var.set("—")
        self.abs_words_var.set("LOW=—  HIGH=—")
        self.angle_var.set("—")
        self.turns_var.set("—")
        self.log("COM-порт закрыт.")

    # --------------------------------------------------------------- JOG
    def servo_on(self):
        if self.autotune_active:
            return

        def job():
            self._require_connection()
            self.client.write_single(REG_P098_SERVO_ENABLE, 1)

        self._run_async(
            job,
            lambda _: self.log("SERVO ON."),
            lambda exc: messagebox.showerror("SERVO ON", str(exc))
        )

    def servo_off(self):
        if self.pid_active:
            self.stop_pid()
        if self.autotune_active:
            self.stop_autotune()

        def job():
            self._require_connection()
            try:
                self.client.write_single(REG_JOG_COMMAND, CMD_STOP)
            finally:
                self.client.write_single(REG_P098_SERVO_ENABLE, 0)

        self._run_async(
            job,
            lambda _: self.log("SERVO OFF."),
            lambda exc: messagebox.showerror("SERVO OFF", str(exc))
        )

    def apply_speed(self):
        if self.pid_active or self.autotune_active:
            messagebox.showwarning(
                "Скорость",
                "Сначала остановите PID/автонастройку."
            )
            return

        try:
            speed = self._get_speed()
        except Exception as exc:
            messagebox.showerror("Скорость", str(exc))
            return

        def job():
            self._require_connection()
            self.client.write_single(REG_P076_JOG_SPEED, speed)
            return speed

        self._run_async(
            job,
            lambda value: self.log(f"P-076 = {value} rpm."),
            lambda exc: messagebox.showerror("Скорость", str(exc))
        )

    def start_motion(self):
        if self.pid_active or self.autotune_active:
            messagebox.showwarning(
                "JOG",
                "Сначала остановите PID/автонастройку."
            )
            return

        try:
            speed = self._get_speed()
            if speed <= 0:
                raise ValueError("Скорость должна быть > 0.")
            direction = self.direction_var.get()
        except Exception as exc:
            messagebox.showerror("Пуск", str(exc))
            return

        command = CMD_FORWARD if direction == "forward" else CMD_REVERSE

        def job():
            self._require_connection()
            self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
            time.sleep(0.05)
            self.client.write_single(REG_P098_SERVO_ENABLE, 1)
            time.sleep(0.06)
            self.client.write_single(REG_P076_JOG_SPEED, speed)
            self.client.write_single(REG_JOG_COMMAND, command)

        self._run_async(
            job,
            lambda _: self.log(
                f"JOG START: {direction}, {speed} rpm."
            ),
            lambda exc: messagebox.showerror("Пуск", str(exc))
        )

    def stop_motion(self):
        if self.pid_active:
            self.stop_pid()
            return
        if self.autotune_active:
            self.stop_autotune()
            return

        def job():
            self._require_connection()
            self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)

        self._run_async(
            job,
            lambda _: self.log("JOG HOLD / 0 speed."),
            lambda exc: messagebox.showerror("STOP", str(exc))
        )

    # ---------------------------------------------------------- PID control
    def _get_pid_settings(self):
        target = parse_float(self.pid_target_var.get())
        kp = parse_float(self.kp_var.get())
        ki = parse_float(self.ki_var.get())
        kd = parse_float(self.kd_var.get())
        tolerance = parse_float(self.pid_tol_var.get())
        min_rpm = parse_float(self.pid_min_rpm_var.get())
        max_rpm = parse_float(self.pid_max_rpm_var.get())

        if not 0.0 <= target <= 360.0:
            raise ValueError("Целевой угол должен быть в диапазоне 0…360°.")
        target = normalize_angle_360(target)

        if min(kp, ki, kd) < 0:
            raise ValueError("Kp/Ki/Kd должны быть >= 0.")
        if tolerance <= 0:
            raise ValueError("Допуск должен быть > 0°.")
        if min_rpm < 0 or max_rpm <= 0 or min_rpm > max_rpm:
            raise ValueError("Проверьте Min/Max rpm.")
        if max_rpm > 32000:
            raise ValueError("Max rpm > 32000.")

        self._get_cpr()

        return {
            "target": target,
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "tolerance": tolerance,
            "min_rpm": min_rpm,
            "max_rpm": max_rpm,
        }

    def start_pid(self):
        if self.pid_active:
            return
        if self.autotune_active:
            messagebox.showwarning(
                "PID",
                "Дождитесь окончания автонастройки."
            )
            return
        if not self.client.is_open:
            messagebox.showerror("PID", "Сначала подключитесь к приводу.")
            return
        if self.latest_abs_raw is None:
            messagebox.showerror("PID", "Нет данных энкодера.")
            return

        try:
            settings = self._get_pid_settings()
        except Exception as exc:
            messagebox.showerror("PID", str(exc))
            return

        self._save_settings()
        self.pid_stop_event.clear()
        self.pid_active = True
        self.pid_status_var.set("PID запуск...")

        self.pid_thread = threading.Thread(
            target=self._pid_worker,
            args=(settings,),
            daemon=True
        )
        self.pid_thread.start()

    def stop_pid(self):
        if self.pid_active:
            self.pid_stop_event.set()
            self.pid_status_var.set("PID остановка...")

    def _pid_worker(self, s):
        integral = 0.0
        last_t = time.monotonic()
        last_direction = None
        last_speed_cmd = None
        holding = False

        if s["ki"] > 0:
            integral_limit = s["max_rpm"] / s["ki"]
        else:
            integral_limit = 0.0

        try:
            self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
            self.client.write_single(REG_P098_SERVO_ENABLE, 1)

            while not self.pid_stop_event.is_set():
                raw, low, high = self.read_absolute_raw()
                speed_raw = self.client.read_holding(REG_ACTUAL_SPEED, 1)[0]
                actual_rpm = u16_to_i16(speed_raw)

                angle = self.counts_to_degrees(raw)
                # Circular position control: 359° -> 1° gives +2°, not -358°.
                error = shortest_angle_error(s["target"], angle)

                now = time.monotonic()
                dt = max(now - last_t, 0.001)
                last_t = now

                if s["ki"] > 0:
                    integral += error * dt
                    integral = max(
                        -integral_limit,
                        min(integral_limit, integral)
                    )
                else:
                    integral = 0.0

                measured_deg_s = actual_rpm * 6.0
                if self.invert_encoder_var.get():
                    measured_deg_s = -measured_deg_s

                output = (
                    s["kp"] * error
                    + s["ki"] * integral
                    - s["kd"] * measured_deg_s
                )
                output = max(-s["max_rpm"], min(s["max_rpm"], output))

                if abs(error) <= s["tolerance"]:
                    if not holding:
                        self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
                    holding = True
                    last_direction = None
                    last_speed_cmd = 0
                    command_rpm = 0.0
                    status = "В позиции / HOLD"
                else:
                    if holding and abs(error) < s["tolerance"] * 1.5:
                        command_rpm = 0.0
                        status = "В позиции / HOLD"
                    else:
                        holding = False
                        if abs(output) < s["min_rpm"]:
                            output = s["min_rpm"] if error > 0 else -s["min_rpm"]

                        direction = "forward" if output > 0 else "reverse"
                        rpm = max(s["min_rpm"], min(s["max_rpm"], abs(output)))
                        rpm_int = max(1, int(round(rpm)))

                        if last_direction is not None and direction != last_direction:
                            self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
                            time.sleep(0.04)

                        if rpm_int != last_speed_cmd:
                            self.client.write_single(REG_P076_JOG_SPEED, rpm_int)
                            last_speed_cmd = rpm_int

                        if direction != last_direction:
                            self.client.write_single(
                                REG_JOG_COMMAND,
                                CMD_FORWARD if direction == "forward" else CMD_REVERSE
                            )
                            last_direction = direction

                        command_rpm = rpm_int if direction == "forward" else -rpm_int
                        status = "PID активен"

                payload = {
                    "raw": raw,
                    "low": low,
                    "high": high,
                    "angle": angle,
                    "error": error,
                    "actual_rpm": actual_rpm,
                    "output": command_rpm,
                    "status": status,
                }
                self.ui_queue.put(("success", self._pid_ui_update, payload))
                time.sleep(0.06)

        except Exception as exc:
            self.ui_queue.put(("failure", self._pid_failed, exc))
        finally:
            try:
                if self.client.is_open:
                    self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
            except Exception:
                pass

            self.pid_active = False
            self.pid_stop_event.clear()
            self.ui_queue.put(("success", self._pid_finished, None))

    def _pid_ui_update(self, p):
        self.latest_abs_raw = int(p["raw"])
        self.abs_position_var.set(str(p["raw"]))
        self.abs_words_var.set(f"LOW={p['low']}  HIGH={p['high']}")
        self.angle_var.set(f"{p['angle']:.3f}°")
        try:
            self.turns_var.set(f"{self.counts_to_turns(p['raw']):.4f}")
        except Exception:
            self.turns_var.set("—")
        self.actual_speed_var.set(f"{p['actual_rpm']} rpm")

        if p["actual_rpm"] > 0:
            self.actual_dir_var.set("CCW / +")
        elif p["actual_rpm"] < 0:
            self.actual_dir_var.set("CW / −")
        else:
            self.actual_dir_var.set("STOP")

        self.pid_error_var.set(f"Ошибка: {p['error']:.3f}°")
        self.pid_output_var.set(f"Выход: {p['output']:.1f} rpm")
        self.pid_status_var.set(p["status"])

    def _pid_failed(self, exc):
        self.log(f"PID ERROR: {exc}")
        self.pid_status_var.set("PID ошибка")
        messagebox.showerror("PID", str(exc))

    def _pid_finished(self, _):
        if self.pid_status_var.get() != "PID ошибка":
            self.pid_status_var.set("PID остановлен")
        self.pid_output_var.set("Выход: 0 rpm")

    # ------------------------------------------------------------ AUTOTUNE
    def _get_autotune_settings(self):
        test_rpm = parse_float(self.tune_rpm_var.get())
        test_angle = parse_float(self.tune_angle_var.get())
        profile = self.tune_profile_var.get()

        self._get_cpr()

        if not 1 <= test_rpm <= 200:
            raise ValueError(
                "Для автонастройки тестовую скорость задайте 1..200 rpm."
            )
        if not 2 <= test_angle <= 90:
            raise ValueError(
                "Ход автонастройки должен быть 2..90°."
            )
        if profile not in ("Осторожный", "Нормальный", "Быстрый"):
            raise ValueError("Неизвестный профиль автонастройки.")

        return test_rpm, test_angle, profile

    def start_autotune(self):
        if self.autotune_active:
            return
        if self.pid_active:
            messagebox.showwarning(
                "Автонастройка",
                "Сначала остановите PID."
            )
            return
        if not self.client.is_open:
            messagebox.showerror(
                "Автонастройка",
                "Сначала подключитесь к приводу."
            )
            return

        try:
            test_rpm, test_angle, profile = self._get_autotune_settings()
        except Exception as exc:
            messagebox.showerror("Автонастройка", str(exc))
            return

        if not messagebox.askyesno(
            "Автонастройка PID",
            "Двигатель автоматически повернется вперед примерно на "
            f"{test_angle:.1f}°, затем программа попытается вернуть его "
            "к исходной позиции.\n\n"
            f"Тестовая скорость: {test_rpm:.1f} rpm\n"
            f"Профиль: {profile}\n\n"
            "Механизм свободен для такого движения?"
        ):
            return

        self.autotune_stop_event.clear()
        self.autotune_active = True
        self.autotune_status_var.set("Автонастройка: подготовка...")
        self._save_settings()

        self.autotune_thread = threading.Thread(
            target=self._autotune_worker,
            args=(test_rpm, test_angle, profile),
            daemon=True
        )
        self.autotune_thread.start()

    def stop_autotune(self):
        if self.autotune_active:
            self.autotune_stop_event.set()
            self.autotune_status_var.set(
                "Автонастройка: запрошена остановка..."
            )

    def _autotune_progress(self, text):
        self.autotune_status_var.set(text)

    def _autotune_worker(self, test_rpm, test_angle, profile):
        start_raw = None
        motion_sign = 1.0

        try:
            alarm = self.client.read_holding(REG_ALARM, 1)[0]
            if alarm != 0:
                raise RuntimeError(
                    f"Автонастройка запрещена: Alarm code={alarm}."
                )

            start_raw, _, _ = self.read_absolute_raw()
            self.ui_queue.put((
                "success",
                self._autotune_progress,
                f"Старт RAW={start_raw}. Подготовка Servo..."
            ))

            self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
            self.client.write_single(REG_P098_SERVO_ENABLE, 1)
            time.sleep(0.10)
            self.client.write_single(
                REG_P076_JOG_SPEED,
                max(1, int(round(test_rpm)))
            )

            samples = []
            t0 = time.monotonic()

            self.ui_queue.put((
                "success",
                self._autotune_progress,
                "Тест: вращение вперед и измерение скоростного отклика..."
            ))

            self.client.write_single(REG_JOG_COMMAND, CMD_FORWARD)

            while not self.autotune_stop_event.is_set():
                now = time.monotonic()
                elapsed = now - t0

                raw, low, high = self.read_absolute_raw()
                speed_raw = self.client.read_holding(REG_ACTUAL_SPEED, 1)[0]
                speed = u16_to_i16(speed_raw)

                delta_deg = self.delta_counts_to_degrees(raw, start_raw)
                samples.append((elapsed, abs(float(speed)), delta_deg))

                self.latest_abs_raw = raw
                self.ui_queue.put((
                    "success",
                    self._autotune_live_ui,
                    {
                        "raw": raw,
                        "low": low,
                        "high": high,
                        "delta": delta_deg,
                        "speed": speed,
                        "text": (
                            f"Тест: Δ={delta_deg:.2f}°, "
                            f"speed={speed} rpm"
                        ),
                    }
                ))

                if abs(delta_deg) >= test_angle:
                    break

                # Защита от слишком долгого теста.
                if elapsed > 5.0:
                    break

                time.sleep(0.04)

            self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
            time.sleep(0.15)

            if self.autotune_stop_event.is_set():
                raise RuntimeError("Автонастройка остановлена пользователем.")

            if len(samples) < 5:
                raise RuntimeError("Слишком мало измерений для автонастройки.")

            final_delta = samples[-1][2]
            if abs(final_delta) < max(1.0, test_angle * 0.25):
                raise RuntimeError(
                    "Двигатель почти не изменил положение. "
                    "Проверьте Servo/JOG/энкодер."
                )

            motion_sign = 1.0 if final_delta > 0 else -1.0

            # Оценка установившейся скорости.
            tail = [s[1] for s in samples[-min(6, len(samples)):]]
            steady_speed = statistics.median(tail)

            if steady_speed < 0.5:
                raise RuntimeError(
                    "Скорость слишком мала для идентификации."
                )

            gain_speed = steady_speed / test_rpm
            gain_speed = max(0.05, min(gain_speed, 2.0))

            threshold10 = 0.10 * steady_speed
            threshold63 = 0.632 * steady_speed

            t10 = None
            t63 = None
            for t, speed_abs, _ in samples:
                if t10 is None and speed_abs >= threshold10:
                    t10 = t
                if t63 is None and speed_abs >= threshold63:
                    t63 = t

            if t10 is None:
                t10 = 0.0

            if t63 is None:
                tau = max(0.10, samples[-1][0] / 3.0)
            else:
                tau = max(0.05, t63 - t10)

            delay = max(0.0, t10)

            self.ui_queue.put((
                "success",
                self._autotune_progress,
                (
                    f"Идентификация: Kspeed={gain_speed:.3f}, "
                    f"tau={tau:.3f}s, delay={delay:.3f}s. "
                    "Возврат к исходному углу..."
                )
            ))

            # -------------------- Return to start
            return_speed = max(1, int(round(min(test_rpm, 15))))
            self.client.write_single(REG_P076_JOG_SPEED, return_speed)
            self.client.write_single(REG_JOG_COMMAND, CMD_REVERSE)

            return_t0 = time.monotonic()
            previous_abs_delta = abs(final_delta)
            worsening_count = 0

            while not self.autotune_stop_event.is_set():
                raw, low, high = self.read_absolute_raw()
                delta_deg = self.delta_counts_to_degrees(raw, start_raw)
                abs_delta = abs(delta_deg)

                speed_raw = self.client.read_holding(REG_ACTUAL_SPEED, 1)[0]
                speed = u16_to_i16(speed_raw)

                self.latest_abs_raw = raw
                self.ui_queue.put((
                    "success",
                    self._autotune_live_ui,
                    {
                        "raw": raw,
                        "low": low,
                        "high": high,
                        "delta": delta_deg,
                        "speed": speed,
                        "text": (
                            f"Возврат: Δ={delta_deg:.2f}°, "
                            f"speed={speed} rpm"
                        ),
                    }
                ))

                if abs_delta <= 0.8:
                    break

                # На подходе уменьшаем скорость.
                if abs_delta <= max(3.0, test_angle * 0.25):
                    self.client.write_single(REG_P076_JOG_SPEED, 2)

                # Если обратная команда явно уводит дальше — остановка.
                if abs_delta > previous_abs_delta + 0.7:
                    worsening_count += 1
                else:
                    worsening_count = max(0, worsening_count - 1)

                if worsening_count >= 4:
                    raise RuntimeError(
                        "Возврат уводит вал от исходной позиции. "
                        "Проверьте направление JOG."
                    )

                previous_abs_delta = abs_delta

                if time.monotonic() - return_t0 > 7.0:
                    raise RuntimeError(
                        "Не удалось вернуть вал к исходному углу за 7 секунд."
                    )

                time.sleep(0.05)

            self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)

            if self.autotune_stop_event.is_set():
                raise RuntimeError("Автонастройка остановлена пользователем.")

            # -------------------- Gain calculation
            # Identified process:
            # actual speed ~= Kspeed * command_rpm
            # angle_rate[deg/s] ~= 6*Kspeed*command_rpm
            process_k = 6.0 * gain_speed

            profile_cfg = {
                "Осторожный": {
                    "min_ts": 5.0,
                    "factor": 14.0,
                    "kp_scale": 0.80,
                    "ki_scale": 0.15,
                    "kd_scale": 0.50,
                    "max_pid_rpm": max(10.0, min(30.0, test_rpm * 2.0)),
                },
                "Нормальный": {
                    "min_ts": 3.0,
                    "factor": 10.0,
                    "kp_scale": 1.00,
                    "ki_scale": 0.25,
                    "kd_scale": 0.80,
                    "max_pid_rpm": max(15.0, min(60.0, test_rpm * 3.0)),
                },
                "Быстрый": {
                    "min_ts": 2.0,
                    "factor": 7.0,
                    "kp_scale": 1.20,
                    "ki_scale": 0.40,
                    "kd_scale": 1.00,
                    "max_pid_rpm": max(20.0, min(100.0, test_rpm * 4.0)),
                },
            }[profile]

            ts = max(
                profile_cfg["min_ts"],
                profile_cfg["factor"] * (tau + delay)
            )
            zeta = 0.90

            # PI design for an integrating position plant.
            kp_base = 8.0 / (process_k * ts)
            ki_base = 16.0 / (
                (zeta ** 2) * (ts ** 2) * process_k
            )

            # Mild speed-feedback damping. Kd multiplies measured deg/s.
            kd_base = kp_base * tau

            kp = kp_base * profile_cfg["kp_scale"]
            ki = ki_base * profile_cfg["ki_scale"]
            kd = kd_base * profile_cfg["kd_scale"]

            # Conservative clamps for a Windows/Modbus supervisory loop.
            kp = max(0.001, min(kp, 5.0))
            ki = max(0.0, min(ki, 2.0))
            kd = max(0.0, min(kd, 1.0))

            result = {
                "kp": kp,
                "ki": ki,
                "kd": kd,
                "gain_speed": gain_speed,
                "tau": tau,
                "delay": delay,
                "ts": ts,
                "max_pid_rpm": profile_cfg["max_pid_rpm"],
                "profile": profile,
                "final_delta": final_delta,
            }

            self.ui_queue.put((
                "success",
                self._autotune_success,
                result
            ))

        except Exception as exc:
            try:
                if self.client.is_open:
                    self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
            except Exception:
                pass

            self.ui_queue.put((
                "failure",
                self._autotune_failed,
                exc
            ))
        finally:
            self.autotune_active = False
            self.autotune_stop_event.clear()

    def _autotune_live_ui(self, p):
        self.latest_abs_raw = p["raw"]
        self.abs_position_var.set(str(p["raw"]))
        self.abs_words_var.set(
            f"LOW={p['low']}  HIGH={p['high']}"
        )

        try:
            self.angle_var.set(
                f"{self.counts_to_degrees(p['raw']):.3f}°"
            )
            self.turns_var.set(
                f"{self.counts_to_turns(p['raw']):.4f}"
            )
        except Exception:
            pass

        self.actual_speed_var.set(f"{p['speed']} rpm")
        self.autotune_status_var.set(p["text"])

    def _autotune_success(self, r):
        self.kp_var.set(f"{r['kp']:.6f}")
        self.ki_var.set(f"{r['ki']:.6f}")
        self.kd_var.set(f"{r['kd']:.6f}")
        self.pid_max_rpm_var.set(f"{r['max_pid_rpm']:.1f}")

        text = (
            "Автонастройка завершена. "
            f"Kp={r['kp']:.6f}, Ki={r['ki']:.6f}, Kd={r['kd']:.6f}; "
            f"Kspeed={r['gain_speed']:.3f}; tau={r['tau']:.3f}s; "
            f"delay={r['delay']:.3f}s; расчетный Ts≈{r['ts']:.2f}s. "
            "Коэффициенты записаны в поля PID, но запуск PID выполняется отдельно."
        )
        self.autotune_status_var.set(text)
        self.log(text)
        self._save_settings()

        messagebox.showinfo(
            "Автонастройка PID",
            "Автонастройка завершена.\n\n"
            f"Kp = {r['kp']:.6f}\n"
            f"Ki = {r['ki']:.6f}\n"
            f"Kd = {r['kd']:.6f}\n"
            f"Рекомендуемый Max rpm = {r['max_pid_rpm']:.1f}\n\n"
            "Коэффициенты уже перенесены в PID.\n"
            "Перед START PID проверьте целевой угол."
        )

    def _autotune_failed(self, exc):
        text = f"Автонастройка остановлена: {exc}"
        self.autotune_status_var.set(text)
        self.log(text)
        messagebox.showerror("Автонастройка PID", str(exc))

    # ------------------------------------------------------------ Monitor
    def _poll_tick(self):
        if (
            self.polling
            and self.client.is_open
            and not self.poll_in_flight
            and not self.pid_active
            and not self.autotune_active
        ):
            self.poll_in_flight = True

            def job():
                result = {
                    "speed": None,
                    "speed_error": None,
                    "alarm": None,
                    "alarm_error": None,
                    "raw": None,
                    "low": None,
                    "high": None,
                    "abs_error": None,
                }

                try:
                    speed_raw = self.client.read_holding(
                        REG_ACTUAL_SPEED, 1
                    )[0]
                    result["speed"] = u16_to_i16(speed_raw)
                except Exception as exc:
                    result["speed_error"] = str(exc)

                try:
                    result["alarm"] = self.client.read_holding(
                        REG_ALARM, 1
                    )[0]
                except Exception as exc:
                    result["alarm_error"] = str(exc)

                try:
                    raw, low, high = self.read_absolute_raw()
                    result["raw"] = raw
                    result["low"] = low
                    result["high"] = high
                except Exception as exc:
                    result["abs_error"] = str(exc)

                return result

            def ok(r):
                self.poll_in_flight = False

                if r["speed_error"] is None:
                    speed = r["speed"]
                    self.actual_speed_var.set(f"{speed} rpm")
                    if speed > 0:
                        self.actual_dir_var.set("CCW / +")
                    elif speed < 0:
                        self.actual_dir_var.set("CW / −")
                    else:
                        self.actual_dir_var.set("STOP")

                if r["alarm_error"] is None:
                    self.alarm_var.set(str(r["alarm"]))

                if r["abs_error"] is None:
                    self.latest_abs_raw = r["raw"]
                    self.abs_position_var.set(str(r["raw"]))
                    self.abs_words_var.set(
                        f"LOW={r['low']}  HIGH={r['high']}"
                    )
                    try:
                        self.angle_var.set(
                            f"{self.counts_to_degrees(r['raw']):.3f}°"
                        )
                        self.turns_var.set(
                            f"{self.counts_to_turns(r['raw']):.4f}"
                        )
                    except Exception:
                        self.angle_var.set("CPR ?")
                        self.turns_var.set("—")

            def fail(exc):
                self.poll_in_flight = False
                self.log(f"Мониторинг: {exc}")

            self._run_async(job, ok, fail)

        self.root.after(500, self._poll_tick)

    # --------------------------------------------------------------- Close
    def on_close(self):
        self._save_settings()
        self.polling = False

        if self.pid_active:
            self.pid_stop_event.set()
        if self.autotune_active:
            self.autotune_stop_event.set()

        if self.client.is_open:
            try:
                self.client.write_single(REG_JOG_COMMAND, CMD_HOLD)
                time.sleep(0.03)
                self.client.write_single(REG_P098_SERVO_ENABLE, 0)
            except Exception:
                pass

        self.client.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        style = ttk.Style(root)
        preferred = "aqua" if IS_MAC else "vista"
        if preferred in style.theme_names():
            style.theme_use(preferred)
    except Exception:
        pass

    C30ServoApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
