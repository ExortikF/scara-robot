"""Драйвер сервопривода по Modbus RTU / RS-485 (pymodbus 3.15).

Область применения — осознанно ограниченная:

* чтение и запись параметров ``P-0nn``;
* пуск/останов и задание скорости через регистр команд;
* при наличии регистра обратной связи — медленный программный контур
  позиционирования (:meth:`ServoDrive.move_to`).

Чего здесь **нет и быть не может**: позиционирования штатными средствами
привода. Позиционное задание он принимает только импульсами по входам
PULS±/SIGN± (руководство, гл. 4.2 и 4.3.1); через Modbus эту шину не заменить.
Подробности и альтернатива на PTO S7-1200 — в ``docs/s7-1200_motion.md``.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.exceptions import ModbusException

from . import registers as reg

log = logging.getLogger(__name__)


class DriveError(RuntimeError):
    """Ошибка обмена или отказ привода."""


@dataclass
class SerialConfig:
    """Параметры порта RS-485.

    Значения по умолчанию — типовые для приводов этого класса. Реальные
    берутся из параметров связи привода, сверьте по его панели.
    """

    port: str
    baudrate: int = 19200
    parity: str = "N"       # 'N' | 'E' | 'O'
    stopbits: int = 1
    bytesize: int = 8
    #: Сколько ждать ответа. Считано, а не взято на глаз: при 19200 8E1
    #: самый длинный наш обмен занимает 14 мс на проводе. Прежние 300 мс
    #: давали запас в 21 раз, и это выходило боком — на помехах каждый
    #: сбойный обмен стоил 0.9 с (три попытки), а сбойный обход трёх осей
    #: до 2.7 с. Ось на это время переставала слушаться, а нажатая кнопка
    #: висела. Со 100 мс запас всё ещё семикратный, но сбой стоит втрое
    #: дешевле. Если приводы отвечают медленно, поднимите --timeout.
    timeout: float = 0.1
    retries: int = 3
    device_id: int = 1      # адрес ведомого на шине


@dataclass
class TcpConfig:
    """Параметры Modbus TCP.

    Так подключаются через шлюз Ethernet<->RS-485 (USR-TCP232, Waveshare и
    подобные). Для привода ничего не меняется: на его клеммах всё тот же
    RS-485 и тот же набор регистров — Ethernet только довозит кадры до шлюза.

    Framer при этом другой (SOCKET вместо RTU), но pymodbus выбирает его сам.
    Некоторые шлюзы работают в «прозрачном» режиме и ждут RTU-кадр внутри
    TCP — тогда нужен ``framer_rtu=True``.
    """

    host: str
    port: int = 502
    timeout: float = 1.0
    retries: int = 3
    device_id: int = 1
    framer_rtu: bool = False


@dataclass
class DriveConfig:
    """Привязка драйвера к конкретной установке.

    Указывается ровно один транспорт: ``serial`` или ``tcp``.
    """

    serial: SerialConfig | None = None
    tcp: TcpConfig | None = None

    @property
    def transport(self) -> "SerialConfig | TcpConfig":
        if (self.serial is None) == (self.tcp is None):
            raise ValueError("задайте ровно один транспорт: serial или tcp")
        return self.serial or self.tcp  # type: ignore[return-value]

    @property
    def device_id(self) -> int:
        return self.transport.device_id

    #: Регистр 16-битного слова команд. По умолчанию — из снятого дампа.
    cmd_register: int = reg.CMD_REGISTER
    cmd_stop: int = reg.CMD_STOP
    cmd_forward: int = reg.CMD_FORWARD
    cmd_reverse: int = reg.CMD_REVERSE

    #: Параметр задания скорости JOG (номер nn в ``P-0nn``), единицы — об/мин.
    speed_param: int = reg.P_JOG_SPEED

    #: Максимум задания скорости, об/мин. Жёсткий предел: больше не запишем.
    #: Номинал мотора лежит в P-207 (по умолчанию 2500) — сверьтесь с ним.
    speed_limit: int = 1000

    #: Регистр текущего положения (32 бита, два подряд идущих регистра:
    #: 0x101F младшее слово, 0x1020 старшее).
    position_register: int | None = reg.REG_ABS_POS_LOW
    position_word_order: str = "low_first"  # 'low_first' | 'high_first'

    #: Импульсов энкодера на оборот вала — для пересчёта в обороты/градусы.
    counts_per_rev: int | None = None

    #: Ограничители программного контура позиционирования.
    position_tolerance: int = 50      # импульсов
    position_kp: float = 0.05         # задание скорости на импульс ошибки
    position_poll_s: float = 0.05     # период опроса контура
    position_timeout_s: float = 30.0


class ServoDrive:
    """Синхронный клиент привода. Используется как контекстный менеджер.

    >>> cfg = DriveConfig(serial=SerialConfig(port="/dev/cu.usbserial-A1"))
    >>> tcp = DriveConfig(tcp=TcpConfig(host="192.168.1.50"))   # через шлюз
    >>> with ServoDrive(cfg) as drive:            # doctest: +SKIP
    ...     drive.write_param(reg.P_CONTROL_MODE, reg.ControlMode.SPEED)
    ...     drive.run(speed=100)
    ...     drive.stop()
    """

    def __init__(self, config: DriveConfig, *, client=None,
                 lock: "threading.Lock | None" = None) -> None:
        """
        ``client`` и ``lock`` передаются, когда на одной шине несколько
        приводов: физический порт один, поэтому клиент общий, а доступ к нему
        сериализуется замком. См. :class:`scara_drive.bus.DriveBus`.
        """
        self.config = config
        self._shared = client is not None
        self._lock = lock or threading.Lock()
        transport = config.transport

        if client is not None:
            self._client = client
            self._where = (f"{transport.host}:{transport.port}"
                           if isinstance(transport, TcpConfig) else transport.port)
            self._last_command: int | None = None
            self._last_speed: int | None = None
            return

        if isinstance(transport, SerialConfig):
            self._client = ModbusSerialClient(
                port=transport.port,
                baudrate=transport.baudrate,
                parity=transport.parity,
                stopbits=transport.stopbits,
                bytesize=transport.bytesize,
                timeout=transport.timeout,
                retries=transport.retries,
            )
            self._where = transport.port
        else:
            kwargs = {}
            if transport.framer_rtu:
                from pymodbus import FramerType
                kwargs["framer"] = FramerType.RTU
            self._client = ModbusTcpClient(
                transport.host,
                port=transport.port,
                timeout=transport.timeout,
                retries=transport.retries,
                **kwargs,
            )
            self._where = f"{transport.host}:{transport.port}"

        # Кэш последних отправленных значений — чтобы не слать их повторно.
        self._last_command: int | None = None
        self._last_speed: int | None = None

    # --- жизненный цикл ---------------------------------------------------

    def connect(self) -> None:
        if self._shared:
            return          # соединение уже открыто владельцем шины
        if not self._client.connect():
            raise DriveError(f"не открылся порт {self.config.serial.port}")
        log.info("порт %s открыт, slave id=%d",
                 self.config.serial.port, self.config.device_id)

    def close(self) -> None:
        if self._shared:
            return          # общим клиентом распоряжается DriveBus
        self._client.close()

    def __enter__(self) -> "ServoDrive":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        # Останов важнее аккуратного закрытия: гасим привод даже на исключении.
        try:
            self.stop()
        except DriveError:
            log.exception("не удалось остановить привод при выходе")
        finally:
            self.close()

    # --- низкий уровень ---------------------------------------------------

    def _read(self, address: int, count: int = 1) -> list[int]:
        try:
            with self._lock:
                rr = self._client.read_holding_registers(
                    address, count=count, device_id=self.config.device_id
                )
        except ModbusException as exc:
            raise DriveError(f"чтение {address}: {exc}") from exc
        if rr.isError():
            raise DriveError(f"чтение {address}: привод вернул ошибку {rr}")
        return list(rr.registers)

    def _write(self, address: int, value: int) -> None:
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"значение {value} не влезает в 16 бит")
        try:
            with self._lock:
                rr = self._client.write_register(
                    address, value, device_id=self.config.device_id
                )
        except ModbusException as exc:
            raise DriveError(f"запись {value} в {address}: {exc}") from exc
        if rr.isError():
            raise DriveError(f"запись {value} в {address}: привод вернул ошибку {rr}")

    # --- параметры P-0nn --------------------------------------------------

    def read_param(self, number: int) -> int:
        """Прочитать параметр ``P-0nn``."""
        value = self._read(reg.param_addr(number))[0]
        log.debug("P-%03d -> %d", number, value)
        return value

    def write_param(self, number: int, value: int) -> None:
        """Записать параметр ``P-0nn``."""
        log.debug("P-%03d <- %d", number, value)
        self._write(reg.param_addr(number), value)

    def read_params(self, first: int, count: int) -> dict[int, int]:
        """Прочитать ``count`` параметров подряд, начиная с ``P-0<first>``."""
        values = self._read(reg.param_addr(first), count)
        return {first + i: v for i, v in enumerate(values)}

    # --- электронный редуктор --------------------------------------------

    def get_electronic_gear(self) -> tuple[int, int]:
        """Вернуть (числитель, знаменатель) эл. редуктора."""
        high = self.read_param(reg.P_GEAR_NUM_HIGH)
        low = self.read_param(reg.P_GEAR_NUM_LOW)
        den = self.read_param(reg.P_GEAR_DEN)
        return high * reg.GEAR_NUM_SCALE + low, den

    def set_electronic_gear(self, numerator: int, denominator: int) -> None:
        """Задать эл. редуктор. Числитель раскладывается на P-028/P-029."""
        if not 1 <= denominator <= 32767:
            raise ValueError("знаменатель эл. редуктора: 1..32767")
        high, low = divmod(numerator, reg.GEAR_NUM_SCALE)
        if high > 32000:
            raise ValueError(f"числитель {numerator} превышает P-028*10000+P-029")
        self.write_param(reg.P_GEAR_NUM_HIGH, high)
        self.write_param(reg.P_GEAR_NUM_LOW, low)
        self.write_param(reg.P_GEAR_DEN, denominator)

    # --- движение ---------------------------------------------------------

    def set_speed(self, speed: int) -> None:
        """Задать скорость (модуль). Направление задаётся в :meth:`run`."""
        limit = self.config.speed_limit
        if not 0 <= speed <= limit:
            raise ValueError(f"скорость {speed} вне 0..{limit}")
        self.write_param(self.config.speed_param, speed)

    def enable(self) -> None:
        """Разрешить привод и встать на удержание нулевой скорости.

        Порядок важен и взят из штатной программы производителя: сначала
        привод переводится в режим JOG командой удержания, и только потом
        включается разрешение. В обратном порядке часть приводов
        разрешение не подхватывает.
        """
        self._write(self.config.cmd_register, reg.CMD_HOLD)
        self._last_command = reg.CMD_HOLD
        self.write_param(reg.P_SERVO_ENABLE, 1)

    def hold(self) -> None:
        """Удержание на нулевой скорости, привод остаётся под током."""
        if self._last_command == reg.CMD_HOLD:
            return
        self._write(self.config.cmd_register, reg.CMD_HOLD)
        self._last_command = reg.CMD_HOLD

    def release(self) -> None:
        """Снять момент: вал можно вращать руками.

        Отличается от :meth:`hold` тем, что там вал остаётся под током и
        держит позицию, а здесь привод отпускает его совсем.
        """
        self._write(self.config.cmd_register, self.config.cmd_stop)
        self.write_param(reg.P_SERVO_ENABLE, 0)
        self._last_command = self.config.cmd_stop
        self._last_speed = None

    def run(self, speed: int | None = None, *, reverse: bool = False) -> None:
        """Пуск. ``speed`` задаётся до подачи команды направления.

        Повторные записи с тем же значением не отправляются: в контуре
        слежения это экономит по транзакции на каждом проходе, а транзакция
        по RS-485 стоит десятки миллисекунд.
        """
        if speed is not None and speed != self._last_speed:
            self.set_speed(speed)
            self._last_speed = speed
        cmd = self.config.cmd_reverse if reverse else self.config.cmd_forward
        if cmd != self._last_command:
            log.debug("пуск %s, команда 0x%04X",
                      "назад" if reverse else "вперёд", cmd)
            self._write(self.config.cmd_register, cmd)
            self._last_command = cmd

    def stop(self) -> None:
        """Останов."""
        log.info("останов")
        self._write(self.config.cmd_register, self.config.cmd_stop)
        self._last_command = self.config.cmd_stop
        self._last_speed = None

    # --- обратная связь по положению --------------------------------------

    def read_position(self) -> int:
        """Текущее положение в импульсах энкодера (знаковое 32-битное)."""
        address = self.config.position_register
        if address is None:
            raise DriveError(
                "position_register не задан в DriveConfig — адрес регистра "
                "положения не входит в снятый дамп, возьмите его из "
                "руководства или из захвата обмена"
            )
        low, high = self._read(address, 2)
        if self.config.position_word_order == "high_first":
            low, high = high, low
        raw = (high << 16) | low
        return raw - (1 << 32) if raw & (1 << 31) else raw

    def move_to(self, target: int, *, speed: int, settle_s: float = 0.2) -> int:
        """Программный контур позиционирования: крутить, пока не дойдём.

        Пропорциональный регулятор поверх скоростного режима. Точность и
        динамика ограничены периодом обмена по RS-485 (десятки мс на цикл),
        поэтому годится для медленных доводок и наладки, но **не** для
        траекторного движения SCARA — для него нужен PTO, см.
        ``docs/s7-1200_motion.md``.

        Возвращает фактическое положение после останова.
        """
        cfg = self.config
        deadline = time.monotonic() + cfg.position_timeout_s
        log.info("move_to %d, скорость до %d", target, speed)

        try:
            while True:
                error = target - self.read_position()
                if abs(error) <= cfg.position_tolerance:
                    break
                if time.monotonic() > deadline:
                    raise DriveError(
                        f"move_to({target}): таймаут {cfg.position_timeout_s} с, "
                        f"остаточная ошибка {error} имп."
                    )
                command = min(speed, max(1, int(abs(error) * cfg.position_kp)))
                self.run(command, reverse=error < 0)
                time.sleep(cfg.position_poll_s)
        finally:
            self.stop()

        time.sleep(settle_s)
        actual = self.read_position()
        log.info("move_to %d завершён, факт %d (ошибка %d)",
                 target, actual, target - actual)
        return actual


class C30AngleIO:
    """Адаптер :class:`ServoDrive` под интерфейс ``DriveIO`` из :mod:`.goto`.

    Переводит сырые отсчёты энкодера в градусы и обратно, пряча от алгоритма
    выхода в точку всё, что специфично для C30: два регистра положения,
    программный ноль, инверсию и разрешение привода.

    Программный ноль (``zero_offset_counts``) и число отсчётов на оборот
    берутся теми же, что у GUI, — значения совместимы с
    ``gui/c30_servo_settings.json``.
    """

    def __init__(self, drive: "ServoDrive", *, counts_per_rev: int = 131072,
                 zero_offset_counts: int = 0, invert: bool = False) -> None:
        if counts_per_rev <= 0:
            raise ValueError("counts_per_rev должно быть положительным")
        self.drive = drive
        self.counts_per_rev = counts_per_rev
        self.zero_offset_counts = zero_offset_counts
        self.invert = invert

    def enable(self) -> None:
        """Разрешить привод и встать на удержание нулевой скорости."""
        self.drive._write(reg.CMD_REGISTER, reg.CMD_HOLD)
        self.drive.write_param(reg.P_SERVO_ENABLE, 1)

    def read_counts(self) -> int:
        """Абсолютное многооборотное положение в отсчётах, знаковое 32 бита.

        Сборка 32 бит живёт в :meth:`ServoDrive.read_position` в одном
        экземпляре. Второй копии здесь быть не должно: она игнорировала бы
        ``position_word_order``, и при обратном порядке слов две части
        программы читали бы у одного привода разное положение.
        """
        return self.drive.read_position()

    def read_angle(self) -> float:
        """Механический угол одного оборота, 0 <= a < 360."""
        delta = self.read_counts() - self.zero_offset_counts
        if self.invert:
            delta = -delta
        return (delta * 360.0 / self.counts_per_rev) % 360.0

    def set_zero_here(self) -> int:
        """Принять текущее положение за 0°. Возвращает новый offset."""
        self.zero_offset_counts = self.read_counts()
        log.info("программный ноль: offset RAW=%d", self.zero_offset_counts)
        return self.zero_offset_counts

    def read_alarm(self) -> int:
        return self.drive._read(reg.REG_ALARM, 1)[0]

    # --- интерфейс DriveIO ---

    def set_speed(self, rpm: int) -> None:
        self.drive.set_speed(int(rpm))

    def run(self, *, reverse: bool) -> None:
        # Инверсия энкодера означает, что «вперёд» по углу — это «назад» по валу.
        self.drive.run(reverse=reverse != self.invert)

    def hold(self) -> None:
        """Останов с сохранением разрешения — вал остаётся под моментом."""
        self.drive._write(reg.CMD_REGISTER, reg.CMD_HOLD)
