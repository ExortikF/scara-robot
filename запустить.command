#!/bin/bash
# Запуск управления рукой.
#
#   ./запустить.command              с приводами и камерой
#   ./запустить.command --simulate   без железа, посмотреть
cd "$(dirname "$0")"

if [ ! -x ".venv/bin/python" ]; then
    echo "Окружение не создано. Сначала: ./установить.command"
    exit 1
fi

exec .venv/bin/python tools/arm_control.py "$@"
