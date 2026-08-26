"""Генератор импульсов STEP/DIR на Raspberry Pi Pico (MicroPython).

Импульсы делает блок PIO — отдельный аппаратный автомат внутри RP2040. Он
тактируется от кварца и не зависит от того, чем занят процессор и когда придёт
следующий пакет по USB. Процессор только заряжает PIO очередным отрезком.

Протокол — построчный текст через USB-serial, чтобы отлаживать можно было
обычным терминалом:

    M <ось> <шаги> <частота>   отрезок; знак шагов = направление
    P                          текущие положения
    S                          немедленный останов
    ?                          версия

Ответы: ``ok``, ``ok <данные>`` или ``err <причина>``.

Разгон делает ноутбук: он режет движение на отрезки с разной частотой и шлёт
их подряд. Прошивка намеренно тупая — вся логика на стороне Python.
"""

from machine import Pin
import rp2
import select
import sys

VERSION = "pico_step 1"

# GPIO под оси: (step, dir). Правьте под свою распайку.
AXES = {
    0: (2, 3),
    1: (4, 5),
    2: (6, 7),
    3: (8, 9),
}

# Один шаг PIO-программы занимает 5 тактов автомата (2 + 2 + 1).
CYCLES_PER_STEP = 5

# Глубина очереди отрезков на ось.
QUEUE_DEPTH = 4

# PIO не тактуется медленнее sysclk/65536. Отсюда нижняя граница частоты шагов.
MIN_STEP_HZ = 400
MAX_STEP_HZ = 200_000


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def _step_gen():
    """Выдать N импульсов. N-1 приходит через FIFO."""
    pull(block)              # ждём число шагов от процессора
    mov(x, osr)
    label("loop")
    set(pins, 1)   [1]       # STEP высокий: 2 такта
    set(pins, 0)   [1]       # STEP низкий:  2 такта
    jmp(x_dec, "loop")       # 1 такт; повторяем, пока x != 0
    irq(rel(0))              # отрезок кончился


class Axis:
    def __init__(self, index, step_pin, dir_pin):
        self.index = index
        self.dir = Pin(dir_pin, Pin.OUT, value=0)
        self.position = 0
        self._busy = False
        self._queue = []
        self._step = Pin(step_pin, Pin.OUT, value=0)
        self._sm = rp2.StateMachine(index)
        self._configure(MIN_STEP_HZ)

    def _configure(self, step_hz):
        """Перенастроить автомат на новую частоту.

        У rp2.StateMachine нет отдельного сеттера частоты — она задаётся
        только через init(), поэтому обработчик прерывания вешаем заново.
        """
        self._sm.init(
            _step_gen,
            freq=step_hz * CYCLES_PER_STEP,
            set_base=self._step,
        )
        self._sm.irq(self._done)

    def _done(self, _sm):
        self._busy = False

    @property
    def busy(self):
        return self._busy

    def enqueue(self, steps, step_hz):
        """Поставить отрезок в очередь.

        Очередь нужна для телеуправления: кадры приходят непрерывно, и
        следующий должен быть готов до того, как кончится текущий, иначе
        между отрезками появится пауза и движение станет рваным.
        """
        if len(self._queue) >= QUEUE_DEPTH:
            raise ValueError("очередь полна")
        self._queue.append((steps, step_hz))
        self.service()
        return len(self._queue)

    def service(self):
        """Запустить следующий отрезок, если автомат свободен."""
        if self._busy or not self._queue:
            return
        steps, step_hz = self._queue.pop(0)
        self._start(steps, step_hz)

    def _start(self, steps, step_hz):
        if not MIN_STEP_HZ <= step_hz <= MAX_STEP_HZ:
            raise ValueError(f"частота вне {MIN_STEP_HZ}..{MAX_STEP_HZ}")
        if steps == 0:
            return

        forward = steps > 0
        count = abs(steps)
        self.dir.value(1 if forward else 0)

        self._sm.active(0)
        self._configure(step_hz)
        self._busy = True
        self._sm.active(1)
        self._sm.put(count - 1)   # jmp(x_dec) выполняет x+1 проходов
        self.position += steps

    def stop(self):
        self._sm.active(0)
        self._sm.restart()
        self._queue = []
        self._busy = False


AXIS_OBJECTS = {i: Axis(i, s, d) for i, (s, d) in AXES.items()}


def handle(line):
    parts = line.split()
    if not parts:
        return None
    cmd = parts[0].upper()

    if cmd == "?":
        return f"ok {VERSION}"

    if cmd == "S":
        for axis in AXIS_OBJECTS.values():
            axis.stop()
        return "ok stopped"

    if cmd == "P":
        return "ok " + " ".join(
            f"{i}:{a.position}{'*' if a.busy else ''}"
            for i, a in sorted(AXIS_OBJECTS.items())
        )

    if cmd == "M":
        if len(parts) != 4:
            return "err нужно: M <ось> <шаги> <частота>"
        index, steps, step_hz = (int(p) for p in parts[1:])
        axis = AXIS_OBJECTS.get(index)
        if axis is None:
            return f"err нет оси {index}"
        try:
            queued = axis.enqueue(steps, step_hz)
        except ValueError as exc:
            return f"err {exc}"
        # Ответ несёт глубину очереди: по ней ноутбук понимает, что не
        # успевает, и должен притормозить, а не копить отставание.
        return f"ok q{queued}"

    return f"err неизвестная команда {cmd}"


def main():
    """Неблокирующий цикл.

    Читать stdin блокирующе нельзя: пока ждём строку, некому запустить
    следующий отрезок из очереди, и между кадрами появляются паузы.
    """
    print(f"ok {VERSION}")
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)

    while True:
        if poller.poll(1):                  # 1 мс — не тормозим обслуживание
            line = sys.stdin.readline()
            if line:
                try:
                    reply = handle(line.strip())
                except Exception as exc:    # прошивка не должна падать молча
                    reply = f"err {type(exc).__name__}: {exc}"
                if reply:
                    print(reply)
        for axis in AXIS_OBJECTS.values():
            axis.service()


if __name__ == "__main__":
    main()
