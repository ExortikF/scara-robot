#!/bin/bash
# Управление рукой с камеры RealSense.
#
# Почему не обычный ./.venv/bin/python:
#   * pyrealsense2 собран вручную под python 3.12 и лежит в .venv-rs;
#   * macOS не отдаёт камеру библиотеке без прав root, отсюда sudo;
#   * VDCAssistant держит RGB-интерфейс камеры, его надо согнать.
#
# Берётся только цветной поток. Глубина не запрашивается: она тут не
# нужна, а на USB её отсутствие освобождает половину полосы.
set -e
cd "$(dirname "$0")"

PY=".venv-rs/bin/python"
if [ ! -x "$PY" ]; then
    echo "[ОШИБКА] нет окружения с pyrealsense2: $PY"
    echo "Соберите его или запускайте обычной программой без --realsense."
    exit 1
fi

if ! "$PY" -c "import pyrealsense2" 2>/dev/null; then
    echo "[ОШИБКА] в $PY нет pyrealsense2"
    exit 1
fi

echo "сгоняю VDCAssistant с камеры..."
sudo killall VDCAssistant 2>/dev/null || true

echo "запускаю (потребуется пароль для sudo)..."
exec sudo "$PY" tools/arm_control.py --realsense --confirm "$@"
