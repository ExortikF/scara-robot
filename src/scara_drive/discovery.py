"""Поиск переходника USB<->RS-485 среди последовательных портов.

На macOS у одного физического адаптера всегда два устройства:

* ``/dev/cu.XXX`` — вызываемое (dial-in). При открытии ждёт сигнал DCD,
  которого у переходника нет, поэтому open() может просто зависнуть.
* ``/dev/cu.XXX``  — вызывающее (call-out). **Открывать надо это.**

Отличие древнее, из времён модемов, но живо до сих пор и стабильно съедает
вечер тому, кто про него не знает.
"""

from __future__ import annotations

from dataclasses import dataclass

# Производители чипов, которые почти всегда и есть USB<->RS-485.
KNOWN_CHIPS = {
    0x0403: "FTDI",
    0x1A86: "WCH (CH340/CH341)",
    0x10C4: "Silicon Labs (CP210x)",
    0x067B: "Prolific (PL2303)",
}

# Встроенные порты macOS, которые к делу не относятся.
IGNORED_HINTS = ("Bluetooth", "wlan-debug", "debug-console")


@dataclass(frozen=True)
class Port:
    device: str
    description: str
    vid: int | None
    pid: int | None

    @property
    def chip(self) -> str | None:
        return KNOWN_CHIPS.get(self.vid) if self.vid else None

    @property
    def is_callout(self) -> bool:
        """macOS: /dev/cu.* — то, что нужно открывать."""
        return "/cu." in self.device or not self.device.startswith("/dev/cu.")

    def __str__(self) -> str:
        chip = f" [{self.chip}]" if self.chip else ""
        return f"{self.device}{chip}  {self.description}"


def list_ports(*, adapters_only: bool = True) -> list[Port]:
    """Перечислить порты. По умолчанию — только похожие на USB-адаптеры.

    Возвращает только call-out устройства: dial-in для RS-485 бесполезны.
    """
    from serial.tools import list_ports as _lp

    found = []
    for item in _lp.comports():
        port = Port(item.device, item.description or "", item.vid, item.pid)
        if not port.is_callout:
            continue
        if adapters_only:
            if port.chip is None:
                continue
        elif any(hint in port.device for hint in IGNORED_HINTS):
            continue
        found.append(port)
    return sorted(found, key=lambda p: p.device)


def guess_port() -> str | None:
    """Единственный подходящий адаптер, если он ровно один."""
    candidates = list_ports()
    return candidates[0].device if len(candidates) == 1 else None
