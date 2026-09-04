#!/bin/bash
# Запуск управления рукой с камеры RealSense.
#
# Почему не обычный ./.venv/bin/python:
#   * pyrealsense2 собран вручную под python 3.12 и живёт в соседнем
#     проекте arm_realsense — в окружении scara_robot его нет;
#   * macOS не отдаёт камеру библиотеке без прав root, поэтому sudo;
#   * VDCAssistant держит RGB-интерфейс камеры, его надо согнать.
set -e
cd "$(dirname "$0")"

PY=/Users/macbook/Desktop/projects/arm_realsense/venv/bin/python
if [ ! -x "$PY" ]; then
    echo "[ОШИБКА] нет питона с pyrealsense2: $PY"
    exit 1
fi

echo "сгоняю VDCAssistant с камеры..."
sudo killall VDCAssistant 2>/dev/null || true

echo "запускаю (потребуется пароль для sudo)..."
exec sudo "$PY" tools/arm_control.py --realsense "$@"
