"""Показать переходники USB<->RS-485, воткнутые в ноутбук.

    python examples/ports.py
    python examples/ports.py --all      # вообще все порты
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scara_drive.discovery import list_ports


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--all", action="store_true", help="включая не-USB порты")
    args = p.parse_args()

    ports = list_ports(adapters_only=not args.all)
    if not ports:
        print("переходник не найден.\n")
        print("проверьте по порядку:")
        print("  1. воткнут ли адаптер в ноутбук;")
        print("  2. виден ли он системе:  ioreg -p IOUSB -l -w 0 | grep -i 'USB Product'")
        print("  3. для CH340 на старых macOS нужен драйвер WCH;")
        print("  4. запустите с --all — вдруг чип неизвестного производителя.")
        return 1

    for port in ports:
        print(port)
    print(f"\nнайдено: {len(ports)}")
    print("на macOS открывать нужно /dev/cu.* (не /dev/cu.*)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
