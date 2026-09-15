#!/bin/bash
# Установка одной командой. Двойной щелчок по этому файлу.
#
# Питон выбирается не по свежести, а по пригодности: он должен уметь
# открывать окна. Питон из Homebrew часто собран без Tk, и без этой
# проверки установка проходит целиком и падает на последнем шаге.
#
# Имена переменных латиницей: оболочка кириллицу в них не принимает.
cd "$(dirname "$0")"

chosen=""
for PY in python3.12 python3.13 python3.11 python3.10 python3.14 python3; do
    command -v "$PY" >/dev/null 2>&1 || continue
    if "$PY" -c "import tkinter" >/dev/null 2>&1; then
        chosen="$PY"
        break
    fi
    echo "  $PY есть, но собран без tkinter — пропускаю"
done

if [ -n "$chosen" ]; then
    echo "беру $chosen"
    exec "$chosen" tools/install.py "$@"
fi

echo
echo "Не нашёл питон с поддержкой окон."
echo "Поставьте:  brew install python@3.12 python-tk@3.12"
echo "или скачайте с https://www.python.org/downloads/ — там Tk уже внутри."
exit 1
