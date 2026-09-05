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

# Если прошлый запуск завершился неаккуратно, камера остаётся занята,
# и следующий получает «No device connected». Гасим хвосты заранее.
# Незавершённый прошлый запуск держит камеру, и новый получит
# «failed to set power state» при исправном устройстве.
if pgrep -f "arm_control.py" >/dev/null 2>&1; then
    echo "нахожу незакрытый прошлый запуск, останавливаю..."
    sudo pkill -f "arm_control.py" 2>/dev/null || true
    sleep 1
    # Упрямые добиваем: пока процесс жив, камера занята.
    if pgrep -f "arm_control.py" >/dev/null 2>&1; then
        sudo pkill -9 -f "arm_control.py" 2>/dev/null || true
        sleep 1
    fi
fi

echo "сгоняю VDCAssistant с камеры..."
sudo killall VDCAssistant 2>/dev/null || true
sleep 1

# Кэш режима пишется из-под root и остаётся с правами root. Если его
# потом не удаётся перезаписать, подбор каждый раз начинается заново.
sudo chown "$(id -u)":"$(id -g)" tools/realsense_mode.json 2>/dev/null || true

echo "запускаю (потребуется пароль для sudo)..."

# Микрофон под sudo обычно недоступен: macOS выдаёт доступ приложению, а
# от имени root запрос уходит мимо разрешения и отказывает молча. Камере
# root необходим, микрофону он мешает — поэтому передаём наружу, чтобы
# программа могла сказать об этом внятно, а не молчать.
exec sudo SUDO_LAUNCH=1 "$PY" tools/arm_control.py --realsense --confirm "$@"
