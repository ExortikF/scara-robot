/*
  Проверка концевиков — вывод срабатываний в монитор порта.

  ПОДКЛЮЧЕНИЕ
  Концевик подключается между выводом и GND. Больше ничего не нужно:
  подтяжка включена внутри микроконтроллера (INPUT_PULLUP), поэтому
  внешний резистор не требуется.

      вывод D2 ──────┐
                     ├── концевик
      GND    ────────┘

  Вывод при разомкнутом концевике подтянут к питанию и читается как HIGH.
  Замыкание сажает его на землю, и он читается как LOW. Отсюда инверсия
  в коде: замкнут — это LOW.

  ПОЧЕМУ ТАК, А НЕ НА ПЛЮС
  Через замкнутый контакт течёт всего десяток микроампер, а провод к
  концевику при этом соединён с землёй — он хуже ловит наводки, чем если
  бы висел на питании. Для длинных проводов вдоль силового кабеля это
  заметно.

  НОРМАЛЬНО ЗАКРЫТЫЙ КОНТАКТ
  Если концевик нормально замкнутый (NC), поставьте ACTIVE_LOW = false.
  Для защитных концевиков NC правильнее: обрыв провода выглядит как
  срабатывание, а не как «всё в порядке».

  МОНИТОР ПОРТА
  Инструменты -> Монитор порта, скорость 115200.
*/

// ── настройка ───────────────────────────────────────────────────────────

const uint8_t SWITCH_PINS[] = {2};          // выводы концевиков
const char*   SWITCH_NAMES[] = {"J1 концевик"};

const bool     ACTIVE_LOW   = true;         // true для нормально разомкнутого
const uint16_t DEBOUNCE_MS  = 20;           // подавление дребезга
const bool     MIRROR_TO_LED = true;        // дублировать на светодиод платы

// ── состояние ───────────────────────────────────────────────────────────

const uint8_t COUNT = sizeof(SWITCH_PINS) / sizeof(SWITCH_PINS[0]);

bool     stableState[COUNT];      // подтверждённое состояние, true = замкнут
bool     lastRead[COUNT];         // последнее сырое чтение
uint32_t lastChangeMs[COUNT];     // когда сырое чтение изменилось
uint32_t hits[COUNT];             // сколько раз срабатывал

bool isClosed(uint8_t index) {
  int raw = digitalRead(SWITCH_PINS[index]);
  return ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

void report(uint8_t index, bool closed) {
  uint32_t now = millis();
  Serial.print('[');
  Serial.print(now / 1000);
  Serial.print('.');
  uint16_t ms = now % 1000;
  if (ms < 100) Serial.print('0');
  if (ms < 10)  Serial.print('0');
  Serial.print(ms);
  Serial.print("] ");
  Serial.print(SWITCH_NAMES[index]);
  Serial.print(" (D");
  Serial.print(SWITCH_PINS[index]);
  Serial.print(")  ");
  if (closed) {
    Serial.print("ЗАМКНУТ    срабатывание #");
    Serial.println(hits[index]);
  } else {
    Serial.println("разомкнут");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }   // ждём монитор, но не вечно

  if (MIRROR_TO_LED) pinMode(LED_BUILTIN, OUTPUT);

  Serial.println();
  Serial.println("=== проверка концевиков ===");
  Serial.print("подавление дребезга: ");
  Serial.print(DEBOUNCE_MS);
  Serial.println(" мс");
  Serial.println("замкните концевик — появится строка");
  Serial.println();

  for (uint8_t i = 0; i < COUNT; i++) {
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);
    delay(2);                              // дать подтяжке установиться
    stableState[i] = isClosed(i);
    lastRead[i]    = stableState[i];
    lastChangeMs[i] = millis();
    hits[i] = 0;

    // Стартовое состояние печатаем всегда: если концевик уже нажат при
    // включении, знать об этом важнее, чем ждать первого изменения.
    Serial.print("при старте: ");
    report(i, stableState[i]);
  }
  Serial.println();
}

void loop() {
  bool anyClosed = false;

  for (uint8_t i = 0; i < COUNT; i++) {
    bool now = isClosed(i);

    // Дребезг: отсчёт начинается заново на каждое изменение сырого чтения,
    // и состояние считается подтверждённым, только когда оно продержалось
    // DEBOUNCE_MS без единого дрогнувшего чтения.
    if (now != lastRead[i]) {
      lastRead[i] = now;
      lastChangeMs[i] = millis();
    }

    if (now != stableState[i] && millis() - lastChangeMs[i] >= DEBOUNCE_MS) {
      stableState[i] = now;
      if (now) hits[i]++;
      report(i, now);
    }

    if (stableState[i]) anyClosed = true;
  }

  if (MIRROR_TO_LED) digitalWrite(LED_BUILTIN, anyClosed ? HIGH : LOW);
}
