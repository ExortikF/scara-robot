#!/bin/bash
# Запуск C30 Servo GUI на macOS. Замена start_c30_servo_v5.bat.
# Двойной клик в Finder или ./gui/start_c30_servo.command из терминала.
set -e
cd "$(dirname "$0")/.."

echo "=========================================="
echo "C30 Servo Control v5 — PID Autotune (macOS)"
echo "=========================================="

VENV=".venv"
PY="$VENV/bin/python"

if [ ! -x "$PY" ]; then
    echo "[1/3] Создаю виртуальное окружение..."
    python3 -m venv "$VENV"
fi

echo "[2/3] Проверяю зависимости..."
if ! "$PY" -c "import serial" 2>/dev/null; then
    echo "      ставлю pyserial..."
    "$PY" -m pip install --disable-pip-version-check -q pyserial
fi

# tkinter в venv наследуется от базового python. Если его нет — venv не спасёт.
if ! "$PY" -c "import tkinter" 2>/dev/null; then
    cat <<'MSG'
[ОШИБКА] У этого Python нет tkinter.

Обычная причина на macOS — сборка python без tcl-tk (частый случай у pyenv).
Починка:

    brew install python-tk

Если python стоит через pyenv, доустановите tcl-tk и пересоберите версию:

    brew install tcl-tk
    pyenv install --force $(pyenv version-name)

Затем удалите .venv и запустите этот скрипт заново.
MSG
    exit 1
fi

echo "[3/3] Запускаю..."
exec "$PY" gui/c30_servo_gui.py
