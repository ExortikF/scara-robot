#!/bin/sh
# Снимок рабочей версии: тег в гите и отдельная копия папкой.
#
# Тег даёт точный откат (git checkout <тег>), но им нельзя пользоваться,
# пока правки не зафиксированы. Копия папкой этим не связана: она
# сохраняет ровно то, что сейчас лежит на диске, включая незафиксированное
# и файлы настроек, которых в гите нет. Первое нужно, чтобы вернуться;
# второе — чтобы сравнить «а как было полчаса назад».
#
#     tools/backup.sh "короткое описание"
#
# Копии складываются в ../scara_robot_backups, ВНЕ проекта: иначе
# следующий снимок начнёт копировать предыдущие.
set -e

cd "$(dirname "$0")/.."
root=$(pwd)
stamp=$(date +%Y%m%d_%H%M%S)
note=${1:-снимок}
store="$root/../scara_robot_backups"
dest="$store/$stamp"

mkdir -p "$dest"
rsync -a --exclude '.venv' --exclude '.venv-rs' --exclude '__pycache__' \
      --exclude 'logs' --exclude 'models' --exclude '.git' \
      --exclude 'camera_shots' --exclude 'dist' \
      "$root/" "$dest/"
printf '%s\n' "$note" > "$dest/ЧТО_ЭТО.txt"
git -C "$root" rev-parse HEAD > "$dest/КОММИТ.txt" 2>/dev/null || true

# Тег ставится, только когда дерево чистое: иначе он указывал бы на код,
# которого в этом снимке нет, и откат по нему обманул бы.
if [ -z "$(git -C "$root" status --porcelain --untracked-files=no)" ]; then
    git -C "$root" tag -f "версия-$stamp" -m "$note" >/dev/null
    echo "тег: версия-$stamp"
else
    echo "тег не ставлю: есть незафиксированные правки"
fi

echo "копия: $dest"
du -sh "$dest" | awk '{print "размер: " $1}'
ls -1 "$store" | wc -l | awk '{print "всего снимков: " $1}'
