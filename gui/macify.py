"""Адаптация виндовой сборки C30 Servo GUI под macOS.

Новые версии GUI приходят как самостоятельные файлы под Windows. Вместо того
чтобы каждый раз править их руками, правки описаны здесь и накладываются одной
командой:

    python gui/macify.py ~/Downloads/C30_Servo_GUI_v6/c30_servo_gui_v6.py

Результат кладётся рядом с этим файлом. Оригинал не изменяется.

Что чинится и почему:

1. Список портов. На macOS у адаптера два устройства, и ``/dev/tty.*`` при
   открытии виснет в ожидании DCD, которого у RS-485 нет. Показываем только
   ``/dev/cu.*``, выкидываем Bluetooth, USB-адаптеры поднимаем наверх.
2. Порт по умолчанию ``COM4`` — на маке такого нет.
3. Шрифт ``Segoe UI`` — на маке отсутствует, Tk молча подставляет замену и
   вёрстка разъезжается.
4. Тема ttk ``vista`` -> ``aqua``.
5. ``minsize(1000, 930)`` не влезает в рабочую область MacBook: нижние кнопки
   уходят под Dock.

Если очередная версия поменяет эти места, накладка упадёт с внятным
сообщением — это лучше, чем тихо пропустить правку.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

PLATFORM_PRELUDE = '''import serial
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
'''

PORT_PICKER_OLD = '''        ports = [p.device for p in list_ports.comports()]
        current = self.port_var.get()
        self.port_combo["values"] = ports
        if current in ports:
            self.port_var.set(current)
        elif "COM4" in ports:
            self.port_var.set("COM4")'''

PORT_PICKER_NEW = '''        ports = usable_ports()
        current = self.port_var.get()
        self.port_combo["values"] = ports
        if current in ports:
            self.port_var.set(current)
        elif not IS_MAC and "COM4" in ports:
            self.port_var.set("COM4")'''

THEME_OLD = '''        style = ttk.Style(root)
        if "vista" in style.theme_names():
            style.theme_use("vista")'''

THEME_NEW = '''        style = ttk.Style(root)
        preferred = "aqua" if IS_MAC else "vista"
        if preferred in style.theme_names():
            style.theme_use(preferred)'''

MINSIZE_OLD = '''        self.root.minsize(1000, 930)'''

MINSIZE_NEW = '''        # 930 px не влезают в рабочую область MacBook: окно уезжает под Dock
        # и нижние кнопки становятся недоступны.
        screen_height = self.root.winfo_screenheight()
        self.root.minsize(1000, min(930, max(600, screen_height - 160)))'''


def macify(source: str) -> tuple[str, list[str]]:
    """Наложить правки. Возвращает (текст, список применённых правок)."""
    applied = []

    def replace(text: str, old: str, new: str, name: str) -> str:
        if old not in text:
            raise SystemExit(
                f"не нашёл место для правки «{name}» — видимо, в этой версии "
                f"его переписали. Правьте вручную и обновите gui/macify.py."
            )
        applied.append(name)
        return text.replace(old, new, 1)

    text = replace(source, '''import serial
from serial.tools import list_ports
''', PLATFORM_PRELUDE, "платформенная преамбула")
    text = replace(text, PORT_PICKER_OLD, PORT_PICKER_NEW, "выбор порта")
    text = replace(text, THEME_OLD, THEME_NEW, "тема ttk")
    text = replace(text, MINSIZE_OLD, MINSIZE_NEW, "размер окна")

    text, count = re.subn(r'font=\("Segoe UI", (\d+), "bold"\)',
                          r'font=(UI_FONT, \1, "bold")', text)
    if not count:
        raise SystemExit("не нашёл ни одного шрифта Segoe UI — проверьте версию")
    applied.append(f"шрифты ({count} шт.)")
    return text, applied


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("source", type=Path, help="виндовый .py из архива")
    p.add_argument("-o", "--output", type=Path,
                   help="куда положить (по умолчанию рядом с macify.py)")
    args = p.parse_args()

    if not args.source.is_file():
        raise SystemExit(f"нет такого файла: {args.source}")

    output = args.output or Path(__file__).with_name(args.source.name)
    if output.resolve() == args.source.resolve():
        raise SystemExit("отказываюсь писать поверх оригинала")

    text, applied = macify(args.source.read_text(encoding="utf-8"))
    output.write_text(text, encoding="utf-8")

    print(f"{args.source.name} -> {output}")
    for item in applied:
        print(f"  применено: {item}")
    print(f"\nзапуск:  ./gui/start_c30_servo.command {output.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
