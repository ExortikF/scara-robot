# scara_robot

Управление сервоприводом SCARA по **Modbus RTU / RS-485** (pymodbus 3.15).

## Установка

```
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Что этот драйвер умеет и чего не умеет

| Задача                                   | Через Modbus RS-485 |
|------------------------------------------|---------------------|
| Чтение/запись параметров `P-0nn`         | да                  |
| Пуск / останов / направление             | да                  |
| Задание скорости                         | да                  |
| Настройка электронного редуктора         | да                  |
| **Позиционирование штатное**             | **нет**             |
| Траекторное движение SCARA               | нет                 |

Привод принимает позиционное задание **только импульсами** по входам
PULS±/SIGN± (руководство C30, гл. 4.2 и 4.3.1). Внутреннего многопозиционного
режима у него нет. Modbus — канал настройки и диагностики, не канал траектории.

`ServoDrive.move_to()` закрывает контур положения программно поверх скоростного
режима, но период обмена по RS-485 — десятки миллисекунд на цикл, поэтому это
инструмент наладки и медленных доводок, а не движения робота.

Рабочий вариант для SCARA — PTO S7-1200 на импульсные входы привода:
[docs/s7-1200_motion.md](docs/s7-1200_motion.md) (обзор) и
[docs/pulse_positioning.md](docs/pulse_positioning.md) (расчёт тракта).

Подобрать электронный редуктор, разрешение и потолок скорости сустава:

```bash
.venv/bin/python examples/plan_axes.py --reduction 50
```

## Импульсное управление с ноутбука

Ноутбук импульсы не генерирует — между ним и приводом стоит микроконтроллер.
Почему именно так и как подключать: [docs/realtime_pulses.md](docs/realtime_pulses.md).

```
ноутбук (Python)  --USB-->  Pico (PIO)  --STEP/DIR-->  привод
```

Посмотреть план разгона, ничего не подключая:

```bash
.venv/bin/python examples/pulse_test.py --dry-run --degrees 90
```

## Приехать в заданный угол

```bash
.venv/bin/python examples/goto_angle.py 90 180 270 0
.venv/bin/python examples/goto_angle.py 90 --simulate   # без железа
```

Подъезд уступами с проверкой после остановки — настраивать нечего, PID не
нужен. Как и почему: [docs/goto_angle.md](docs/goto_angle.md).

## Начало работы

Поднять связь по RS-485 — пошагово: [docs/rs485_setup.md](docs/rs485_setup.md).

```bash
.venv/bin/python examples/ports.py              # найти переходник
.venv/bin/python examples/mb.py read P-004      # прочитать регистр
.venv/bin/python examples/mb.py write P-076 100 --verify
```

Найти привод на шине (перебирает скорость, чётность и адрес; читает только
`P-003`, ничего не пишет):

```bash
.venv/bin/python examples/scan_bus.py /dev/cu.usbserial-XXXX --ids 1-32
```

Толчковый прогон на разгруженной оси:

```bash
.venv/bin/python examples/jog.py /dev/cu.usbserial-XXXX --speed 100 --seconds 2
```

## Пример

```python
from scara_drive import DriveConfig, SerialConfig, ServoDrive
from scara_drive import registers as reg

cfg = DriveConfig(SerialConfig(port="/dev/cu.usbserial-XXXX", device_id=1))

with ServoDrive(cfg) as drive:
    print(drive.read_param(reg.P_CONTROL_MODE))
    drive.set_electronic_gear(numerator=10000, denominator=1)
    drive.run(speed=100)
    ...
# при выходе из блока — в т.ч. по исключению и Ctrl-C — вызывается stop()
```

## Что нужно уточнить перед боевым пуском

Часть карты регистров снята с шины, а не взята из руководства:

1. **Параметры связи** — скорость, чётность, адрес ведомого. Даёт `scan_bus.py`.
2. **`P-076`** — предполагается заданием скорости (в дампе писали `100`).
   Единицы неизвестны: об/мин, 0.1 Гц или проценты.
3. **`P-098 = 1`** — назначение неизвестно.
4. **Команда «назад» `0x110D`** — несимметрична команде «вперёд» `0x2222`,
   на битовую маску направления не похожа. Возможна перестановка байт в
   захвате (`0x110D` -> `0x0D11`). Проверять только на разгруженной оси.
5. **Регистр текущего положения** — в дампе отсутствует. Без него
   `move_to()` недоступен: задайте `DriveConfig.position_register`.

Разбор карты регистров — [docs/modbus_registers.md](docs/modbus_registers.md).

## Техника безопасности

- Витая пара, общий GND, терминаторы 120 Ом на обоих концах сегмента.
- `speed_limit` в `DriveConfig` — жёсткий потолок задания, драйвер не даст
  записать больше.
- Аппаратный аварийный останов не заменяется программным `stop()`.
