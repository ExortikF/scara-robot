"""Несколько приводов на одной шине RS-485.

Физический порт один, а приводов на нём несколько — они различаются адресом
ведомого. Поэтому клиент Modbus должен быть общим, а обмен — строго по
очереди: если два потока начнут писать в порт одновременно, кадры
перемешаются, и оба получат мусор вместо ответа.

    bus = DriveBus(SerialConfig(port="/dev/cu.usbserial-XXXX"))
    bus.connect()
    j1 = bus.axis(1)
    j2 = bus.axis(2)
"""

from __future__ import annotations

import logging
import threading

from pymodbus.client import ModbusSerialClient, ModbusTcpClient

from .driver import DriveConfig, DriveError, SerialConfig, ServoDrive, TcpConfig

log = logging.getLogger(__name__)


class DriveBus:
    """Владелец одного соединения и нескольких приводов на нём."""

    def __init__(self, transport: SerialConfig | TcpConfig):
        self.transport = transport
        self._lock = threading.Lock()
        self._axes: dict[int, ServoDrive] = {}
        self._connected = False

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
            self.where = transport.port
        else:
            kwargs = {}
            if transport.framer_rtu:
                from pymodbus import FramerType
                kwargs["framer"] = FramerType.RTU
            self._client = ModbusTcpClient(
                transport.host, port=transport.port,
                timeout=transport.timeout, retries=transport.retries, **kwargs,
            )
            self.where = f"{transport.host}:{transport.port}"

    # --- соединение -------------------------------------------------------

    def connect(self) -> None:
        if self._connected:
            return
        if not self._client.connect():
            raise DriveError(f"не удалось подключиться: {self.where}")
        self._connected = True
        log.info("шина открыта: %s", self.where)

    def close(self) -> None:
        self._client.close()
        self._connected = False

    def __enter__(self) -> "DriveBus":
        self.connect()
        return self

    def __exit__(self, *exc_info) -> None:
        # Гасим все приводы, потом закрываем порт.
        for device_id, axis in self._axes.items():
            try:
                axis.stop()
            except DriveError:
                log.exception("ось %d не остановилась", device_id)
        self.close()

    # --- оси --------------------------------------------------------------

    def axis(self, device_id: int, **overrides) -> ServoDrive:
        """Привод с указанным адресом. Повторный вызов возвращает тот же объект."""
        if device_id in self._axes:
            return self._axes[device_id]

        transport = type(self.transport)(
            **{**vars(self.transport), "device_id": device_id}
        )
        config = DriveConfig(
            serial=transport if isinstance(transport, SerialConfig) else None,
            tcp=transport if isinstance(transport, TcpConfig) else None,
            **overrides,
        )
        drive = ServoDrive(config, client=self._client, lock=self._lock)
        self._axes[device_id] = drive
        return drive

    def stop_all(self) -> None:
        """Остановить все оси. Ошибки не глотаются, но и не прерывают остальных."""
        errors = []
        for device_id, axis in self._axes.items():
            try:
                axis.stop()
            except DriveError as exc:
                errors.append(f"ось {device_id}: {exc}")
        if errors:
            raise DriveError("; ".join(errors))
