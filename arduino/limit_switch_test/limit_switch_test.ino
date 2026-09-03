/*
  Проверка концевиков на ESP32 — вывод срабатываний в монитор порта.

  ═══ ПОДКЛЮЧЕНИЕ ═══

  Схема зависит от того, свободен вывод или на нём висит встроенный
  светодиод. Настраивается одной строкой — SWITCH_TO_GND.

  А. Вывод свободный (GPIO4, 13, 14, 16…19, 21…23, 25…27, 32, 33)
     SWITCH_TO_GND = true, концевик между выводом и GND:

         GPIO ──────┐
                    ├── концевик
         GND  ──────┘

     Работает внутренняя подтяжка к питанию. Разомкнут — HIGH,
     замкнут — LOW.

  Б. Вывод со светодиодом (GPIO2 на большинстве плат)
     SWITCH_TO_GND = false, концевик между выводом и 3.3 В,
     последовательно резистор 330 Ом…1 кОм:

         3.3V ──[330 Ом]── концевик ──┐
                                      ├── GPIO2
         (внутренняя подтяжка вниз) ───┘

     Работает внутренняя подтяжка к земле. Разомкнут — LOW,
     замкнут — HIGH.

  ═══ ПОЧЕМУ НА GPIO2 НУЖНА СХЕМА Б ═══

  К GPIO2 на большинстве плат припаян светодиод через резистор на землю.
  Это постоянная нагрузка. Внутренняя подтяжка вверх слабая, около 45 кОм,
  и перетянуть светодиод не может: вывод садится примерно до 1.5 В при
  пороге единицы 0.75 x 3.3 = 2.48 В. Схема А на таком выводе показывала бы
  «ЗАМКНУТ» всегда.

  В схеме Б замкнутый концевик соединяет вывод с питанием напрямую —
  жёстко, а не через слабый резистор. Светодиод при этом загорается, что
  даже удобно: видно срабатывание без монитора. Резистор 330 Ом ограничивает
  ток через него.

  Внутренняя подтяжка вниз (INPUT_PULLDOWN) есть именно у ESP32; у обычных
  Arduino её нет, там пришлось бы ставить внешний резистор.

  Strapping-пину GPIO2 такая схема не мешает: при включении концевик
  разомкнут, вывод подтянут вниз — это разрешённое состояние.

  ═══ ВЫВОДЫ, КОТОРЫЕ НЕ ГОДЯТСЯ ═══

    GPIO 6…11   заняты флеш-памятью
    GPIO 34…39  только вход, внутренних подтяжек нет вообще —
                нужен внешний резистор 10 кОм

  ═══ МОНИТОР ПОРТА ═══
  Инструменты -> Монитор порта, скорость 115200.
  При старте скетч сам проверяет вывод и говорит, годится ли он.
*/

// ── настройка ───────────────────────────────────────────────────────────

const uint8_t SWITCH_PINS[]  = {2};             // выводы концевиков
const char*   SWITCH_NAMES[] = {"J1 концевик"};

const bool SWITCH_TO_GND = false;   // false = концевик на 3.3 В (схема Б)
                                    // true  = концевик на GND  (схема А)

const bool NORMALLY_CLOSED = false; // true для концевиков типа NC
const uint16_t DEBOUNCE_MS = 20;    // подавление дребезга

// ── производные ─────────────────────────────────────────────────────────

const uint8_t COUNT = sizeof(SWITCH_PINS) / sizeof(SWITCH_PINS[0]);
const uint8_t PIN_MODE = SWITCH_TO_GND ? INPUT_PULLUP : INPUT_PULLDOWN;

#ifdef LED_BUILTIN
  const int LED_PIN = LED_BUILTIN;
#else
  const int LED_PIN = -1;
#endif

bool     stableState[COUNT];
bool     lastRead[COUNT];
uint32_t lastChangeMs[COUNT];
uint32_t hits[COUNT];
bool     ledFree = (LED_PIN >= 0);   // уточняется в setup

bool isClosed(uint8_t index) {
  int raw = digitalRead(SWITCH_PINS[index]);
  bool closed = SWITCH_TO_GND ? (raw == LOW) : (raw == HIGH);
  return NORMALLY_CLOSED ? !closed : closed;
}

/*
  Проверка вывода: подтянуть его сначала вверх, потом вниз и посмотреть,
  идёт ли он за подтяжкой. Свободный вывод идёт за обеими. Если он застрял
  в одном состоянии — на нём либо замкнутый концевик, либо нагрузка вроде
  светодиода, и это надо знать до того, как показания начнут врать.
*/
void probePin(uint8_t pin, const char* name) {
  pinMode(pin, INPUT_PULLUP);
  delay(5);
  bool up = digitalRead(pin) == HIGH;

  pinMode(pin, INPUT_PULLDOWN);
  delay(5);
  bool down = digitalRead(pin) == HIGH;

  Serial.print("  GPIO");
  Serial.print(pin);
  Serial.print(" (");
  Serial.print(name);
  Serial.print("): ");

  if (up && !down) {
    Serial.println("свободен, обе подтяжки работают — вывод годится");
  } else if (!up && !down) {
    Serial.println("прижат к земле");
    Serial.println("      либо концевик сейчас замкнут на GND,");
    Serial.println("      либо на выводе нагрузка (светодиод платы).");
    if (SWITCH_TO_GND) {
      Serial.println("      Со схемой А такой вывод будет врать: поставьте");
      Serial.println("      SWITCH_TO_GND = false и концевик на 3.3 В.");
    } else {
      Serial.println("      Для схемы Б это нормально: так и должно быть");
      Serial.println("      при разомкнутом концевике.");
    }
  } else if (up && down) {
    Serial.println("притянут к питанию");
    Serial.println("      концевик замкнут на 3.3 В или есть внешняя подтяжка.");
  } else {
    Serial.println("ведёт себя странно — проверьте проводку");
  }

  pinMode(pin, PIN_MODE);
  delay(5);
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
  Serial.print(" (GPIO");
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
  while (!Serial && millis() < 3000) { }

  Serial.println();
  Serial.println("=== проверка концевиков (ESP32) ===");
  Serial.print("схема: ");
  Serial.println(SWITCH_TO_GND ? "А — концевик на GND, подтяжка вверх"
                               : "Б — концевик на 3.3 В, подтяжка вниз");
  Serial.print("дребезг: ");
  Serial.print(DEBOUNCE_MS);
  Serial.println(" мс");
  Serial.println();

  Serial.println("проверка выводов:");
  for (uint8_t i = 0; i < COUNT; i++) {
    probePin(SWITCH_PINS[i], SWITCH_NAMES[i]);

    // Светодиод и концевик на одном выводе одновременно невозможны:
    // дублирование пришлось бы делать записью в тот же вывод, с которого
    // читаем. Молча оставлять такой конфликт нельзя.
    if (LED_PIN >= 0 && SWITCH_PINS[i] == LED_PIN) ledFree = false;
  }
  Serial.println();

  if (ledFree) pinMode(LED_PIN, OUTPUT);
  else if (LED_PIN >= 0) {
    Serial.println("светодиод платы на выводе концевика — дублирование");
    Serial.println("выключено. В схеме Б он и так загорается при замыкании.");
    Serial.println();
  }

  for (uint8_t i = 0; i < COUNT; i++) {
    stableState[i]  = isClosed(i);
    lastRead[i]     = stableState[i];
    lastChangeMs[i] = millis();
    hits[i]         = 0;

    // Если концевик уже нажат при включении, знать об этом важнее,
    // чем дожидаться первого изменения.
    Serial.print("при старте: ");
    report(i, stableState[i]);
  }
  Serial.println();
  Serial.println("замкните концевик — появится строка");
  Serial.println("(отправьте любой символ, чтобы повторить состояние)");
  Serial.println();
}

/*
  Монитор порта подключается уже после загрузки платы, поэтому всё
  напечатанное в setup() до него не доходит — выглядит это как «скетч
  ничего не выводит». Любой символ из монитора печатает состояние заново.
*/
void reprintOnRequest() {
  if (!Serial.available()) return;
  while (Serial.available()) Serial.read();
  Serial.println();
  Serial.println("─── текущее состояние ───");
  for (uint8_t i = 0; i < COUNT; i++) report(i, stableState[i]);
  Serial.println();
}

void loop() {
  reprintOnRequest();
  bool anyClosed = false;

  for (uint8_t i = 0; i < COUNT; i++) {
    bool now = isClosed(i);

    // Отсчёт начинается заново на каждое дрогнувшее чтение: состояние
    // считается подтверждённым, только продержавшись DEBOUNCE_MS подряд.
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

  if (ledFree) digitalWrite(LED_PIN, anyClosed ? HIGH : LOW);
}
