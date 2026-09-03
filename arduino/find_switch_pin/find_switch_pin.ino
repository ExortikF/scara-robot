/*
  Поиск вывода: к какому GPIO подключён концевик.

  ЗАЧЕМ
  Надпись на плате и номер в коде — разные вещи. «D2» на шелкографии в
  зависимости от платы означает GPIO4, 15, 16, 21, 25 или 27, а у обычного
  «ESP32 Dev Module» такого имени нет вообще: там выводы зовутся номерами
  GPIO. Скетч выясняет это опытом, а не по документации.

  КАК ПОЛЬЗОВАТЬСЯ
  1. Подключите концевик одним концом к выводу, вторым — к GND
     (или к 3.3 В через резистор 330 Ом, если так удобнее).
  2. Залейте скетч, откройте монитор порта на 115200.
  3. Дождитесь таблицы состояния выводов.
  4. Нажмите и отпустите концевик несколько раз.
  5. Скетч назовёт GPIO, который меняется, — это и есть ваш вывод.

  КАК ЭТО РАБОТАЕТ
  Каждый вывод по очереди подтягивается вверх и вниз. Свободный вывод идёт
  за подтяжкой в обе стороны. Прижатый к земле (замкнутый концевик или
  светодиод платы) остаётся низким при обеих. Притянутый к питанию —
  высоким при обеих. Смена состояния и выдаёт нужный вывод.

  ЧТО НЕ ПРОВЕРЯЕТСЯ
    GPIO 1, 3    заняты обменом с компьютером, трогать нельзя
    GPIO 6…11    заняты флеш-памятью
    GPIO 0       кнопка BOOT на плате, её нажатия сбивали бы поиск
    GPIO 34…39   только вход, внутренних подтяжек нет — определить
                 состояние тем же способом невозможно
*/

#include <string.h>

const uint8_t PINS[] = {
  2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
};
const uint8_t COUNT = sizeof(PINS) / sizeof(PINS[0]);

enum State : uint8_t { FREE, HELD_LOW, HELD_HIGH };
const char* STATE_NAME[] = {"свободен", "прижат к GND", "притянут к 3.3В"};

State current[COUNT];
State baseline[COUNT];
uint32_t changes[COUNT];
uint16_t heldMv[COUNT];        // напряжение на выводе, если он прижат вниз

/*
  Отличить замкнутый концевик от светодиода платы.

  Оба дают на выводе логический ноль, и по digitalRead они неразличимы. Но
  напряжение разное: провод на землю сажает вывод почти в ноль, а светодиод
  с резистором образует с внутренней подтяжкой делитель, и на выводе
  остаётся заметное напряжение — около полутора вольт.

  Меряем АЦП при включённой подтяжке вверх.

    ниже 300 мВ    жёсткое соединение с землёй — это провод или концевик
    выше 800 мВ    нагрузка, почти наверняка светодиод платы
*/
uint16_t measurePulledUp(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  delay(2);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += analogRead(pin);
  // АЦП отдаёт 0…4095 на диапазон примерно 0…3.3 В
  return (uint16_t)((sum / 8) * 3300UL / 4095UL);
}

State probe(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(600);
  bool up = digitalRead(pin) == HIGH;

  pinMode(pin, INPUT_PULLDOWN);
  delayMicroseconds(600);
  bool down = digitalRead(pin) == HIGH;

  if (up && !down) return FREE;        // идёт за подтяжкой — ничего не висит
  if (!up && !down) return HELD_LOW;   // что-то держит на земле
  return HELD_HIGH;                    // что-то держит на питании
}

void printTable() {
  Serial.println("  GPIO   состояние         напряжение   вывод");
  Serial.println("  ─────  ────────────────  ──────────   ─────────────────────");
  for (uint8_t i = 0; i < COUNT; i++) {
    Serial.print("  ");
    if (PINS[i] < 10) Serial.print(' ');
    Serial.print(PINS[i]);
    Serial.print("     ");
    Serial.print(STATE_NAME[current[i]]);
    for (uint8_t s = strlen(STATE_NAME[current[i]]); s < 18; s++) Serial.print(' ');

    if (current[i] == HELD_LOW) {
      Serial.print(heldMv[i]);
      Serial.print(" мВ");
      if (heldMv[i] < 300) {
        Serial.println("      провод на землю (концевик)");
      } else if (heldMv[i] > 800) {
        Serial.println("     нагрузка — светодиод платы");
      } else {
        Serial.println("     непонятно, померьте тестером");
      }
    } else {
      Serial.println();
    }
  }
}

/*
  Сводка печатается ПОВТОРНО, а не один раз при запуске.

  Монитор порта в Arduino IDE подключается уже после того, как плата
  загрузилась и отработала setup(). Всё, что напечатано там, до монитора
  просто не доходит, и выглядит это как «скетч ничего не выводит».
  Поэтому сводка повторяется, пока концевик не найден, и печатается по
  любому символу, отправленному из монитора.
*/
void printSummary() {
  Serial.println();
  Serial.println("═══ поиск вывода концевика ═══");
  Serial.println("нажимайте и отпускайте концевик — назову нужный GPIO");
  Serial.println();
  printTable();

  for (uint8_t i = 0; i < COUNT; i++) {
    if (baseline[i] == HELD_LOW && heldMv[i] < 300) {
      Serial.println();
      Serial.print("GPIO");
      Serial.print(PINS[i]);
      Serial.println(" соединён с землёй напрямую.");
      Serial.println("Если концевик СЕЙЧАС РАЗОМКНУТ — значит провод сидит");
      Serial.println("на этом выводе, и это он вам и нужен.");
      Serial.println("Если ЗАМКНУТ — отпустите и посмотрите, освободится ли.");
    }
  }
  Serial.println();
}

void takeBaseline() {
  for (uint8_t i = 0; i < COUNT; i++) {
    current[i] = probe(PINS[i]);
    baseline[i] = current[i];
    changes[i] = 0;
    heldMv[i] = (current[i] == HELD_LOW) ? measurePulledUp(PINS[i]) : 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);                    // дать порту подняться

  takeBaseline();
  printSummary();
}

uint32_t lastSummaryMs = 0;
bool found = false;

void loop() {
  // Любой символ из монитора — напечатать сводку заново.
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    takeBaseline();
    Serial.println("(состояние снято заново)");
    printSummary();
    lastSummaryMs = millis();
  }

  // Пока вывод не найден, повторять сводку — чтобы её увидел монитор,
  // подключившийся с опозданием.
  if (!found && millis() - lastSummaryMs > 4000) {
    lastSummaryMs = millis();
    printSummary();
  }

  for (uint8_t i = 0; i < COUNT; i++) {
    State now = probe(PINS[i]);
    if (now == current[i]) continue;

    current[i] = now;
    changes[i]++;

    Serial.print(">>> GPIO");
    Serial.print(PINS[i]);
    Serial.print("  ");
    Serial.print(STATE_NAME[now]);
    Serial.print("   (изменений: ");
    Serial.print(changes[i]);
    Serial.print(")");

    if (changes[i] >= 2) {
      Serial.print("   <<< ЭТО ВАШ ВЫВОД");
      found = true;              // сводку больше не повторяем
    }
    Serial.println();

    if (changes[i] == 2) {
      Serial.println();
      Serial.print("    В скетче концевика пишите номер ");
      Serial.print(PINS[i]);
      Serial.println(", а не D2:");
      Serial.print("        const uint8_t SWITCH_PINS[] = {");
      Serial.print(PINS[i]);
      Serial.println("};");
      if (baseline[i] == HELD_LOW) {
        Serial.println("    Вывод изначально прижат к земле — на нём светодиод.");
        Serial.println("        SWITCH_TO_GND = false, концевик на 3.3 В");
      } else {
        Serial.println("        SWITCH_TO_GND = true, концевик на GND");
      }
      Serial.println();
    }
  }
  delay(20);
}
