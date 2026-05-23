#include <BleKeyboard.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// ---------------------------------------------------------
// Piny Clickify - zgodnie z Twoją płytką
// ---------------------------------------------------------
#define PLAY_PIN       7
#define FORWARD_PIN    3
#define PREVIOUS_PIN   18
#define LED_PIN        6
#define BATTERY_PIN    4

// ---------------------------------------------------------
// BLE
// ---------------------------------------------------------
BleKeyboard keyboard("Clickify", "Blough", 100);

// ---------------------------------------------------------
// Parametry
// ---------------------------------------------------------
const unsigned long DEBOUNCE_DELAY = 50;

// Po ilu ms bez połączenia BLE usypiać urządzenie
const unsigned long NO_CONNECTION_SLEEP_MS = 30000;

// Co ile sprawdzać baterię
const unsigned long BATTERY_CHECK_INTERVAL_MS = 15000;

// Progi baterii
const float LOW_BATTERY_VOLTAGE = 3.55;
const float CRITICAL_BATTERY_VOLTAGE = 3.30;

// Dzielnik 200k + 200k
const float BATTERY_DIVIDER_RATIO = 2.0;
const float BATTERY_CALIBRATION = 1.00;

// ---------------------------------------------------------
// Zmienne
// ---------------------------------------------------------
unsigned long lastDebounceTime = 0;
unsigned long lastActivityTime = 0;
unsigned long bootTime = 0;
unsigned long lastBatteryCheckTime = 0;
unsigned long lastLowBatteryBlinkTime = 0;

bool lastPlayState = HIGH;
bool lastForwardState = HIGH;
bool lastPrevState = HIGH;

bool lowBattery = false;
bool criticalBattery = false;

// ---------------------------------------------------------
// Funkcje pomocnicze
// ---------------------------------------------------------

bool anyButtonPressed() {
  return digitalRead(PLAY_PIN) == LOW ||
         digitalRead(FORWARD_PIN) == LOW ||
         digitalRead(PREVIOUS_PIN) == LOW;
}

float readBatteryVoltage() {
  const int samples = 16;
  uint32_t sumMv = 0;

  for (int i = 0; i < samples; i++) {
    sumMv += analogReadMilliVolts(BATTERY_PIN);
    delay(2);
  }

  float adcVoltage = (sumMv / (float)samples) / 1000.0;
  float batteryVoltage = adcVoltage * BATTERY_DIVIDER_RATIO * BATTERY_CALIBRATION;

  return batteryVoltage;
}

int batteryPercentFromVoltage(float voltage) {
  if (voltage >= 4.20) return 100;
  if (voltage <= 3.30) return 0;

  return (int)((voltage - 3.30) * 100.0 / (4.20 - 3.30));
}

void updateBatteryState() {
  float batteryVoltage = readBatteryVoltage();
  int batteryPercent = batteryPercentFromVoltage(batteryVoltage);

  lowBattery = batteryVoltage <= LOW_BATTERY_VOLTAGE;
  criticalBattery = batteryVoltage <= CRITICAL_BATTERY_VOLTAGE;

  keyboard.setBatteryLevel(batteryPercent);

  Serial.print("Bateria: ");
  Serial.print(batteryVoltage, 3);
  Serial.print(" V, ");
  Serial.print(batteryPercent);
  Serial.println(" %");
}

void handleLowBatteryLed() {
  if (!lowBattery) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  unsigned long now = millis();

  if (criticalBattery) {
    // Krytycznie niska bateria: 50 ms co 1 s
    if (now - lastLowBatteryBlinkTime >= 1000) {
      lastLowBatteryBlinkTime = now;
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    // Niska bateria: 20 ms co 2 s
    if (now - lastLowBatteryBlinkTime >= 2000) {
      lastLowBatteryBlinkTime = now;
      digitalWrite(LED_PIN, HIGH);
      delay(20);
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void sendCommandForButton(uint8_t buttonPin) {
  if (!keyboard.isConnected()) {
    return;
  }

  if (buttonPin == PLAY_PIN) {
    Serial.println("Akcja: PLAY/PAUSE");
    keyboard.write(KEY_MEDIA_PLAY_PAUSE);
  }
  else if (buttonPin == FORWARD_PIN) {
    Serial.println("Akcja: NEXT");
    keyboard.write(KEY_MEDIA_NEXT_TRACK);
  }
  else if (buttonPin == PREVIOUS_PIN) {
    Serial.println("Akcja: PREVIOUS");
    keyboard.write(KEY_MEDIA_PREVIOUS_TRACK);
  }

  lastActivityTime = millis();
}

void setupWakeupPinsForLightSleep() {
  // Przyciski są do GND, więc wybudzanie po stanie LOW.
  gpio_wakeup_enable((gpio_num_t)PLAY_PIN, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)FORWARD_PIN, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)PREVIOUS_PIN, GPIO_INTR_LOW_LEVEL);

  esp_sleep_enable_gpio_wakeup();
}

void goToLightSleepWhenDisconnected() {
  Serial.println("Brak polaczenia BLE. Przechodze w light-sleep.");

  digitalWrite(LED_PIN, LOW);
  delay(50);
  Serial.flush();

  setupWakeupPinsForLightSleep();

  esp_light_sleep_start();

  // Po wybudzeniu z light-sleep robimy restart.
  // To jest celowe: BLE startuje wtedy czysto od nowa
  // i nie zostaje w dziwnym stanie po wyjsciu ze sleepa.
  ESP.restart();
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  // Obniżenie taktowania CPU. Zmniejsza pobór w stanie aktywnym.
  // Dla ESP32-C3 80 MHz jest bezpieczne.
  setCpuFrequencyMhz(80);

  pinMode(PLAY_PIN, INPUT_PULLUP);
  pinMode(FORWARD_PIN, INPUT_PULLUP);
  pinMode(PREVIOUS_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BATTERY_PIN, INPUT);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  keyboard.begin();

  bootTime = millis();
  lastActivityTime = millis();
  lastBatteryCheckTime = millis();

  lastPlayState = digitalRead(PLAY_PIN);
  lastForwardState = digitalRead(FORWARD_PIN);
  lastPrevState = digitalRead(PREVIOUS_PIN);

  updateBatteryState();

  Serial.println("Clickify gotowy. Szukaj mnie w Bluetooth.");
}

// ---------------------------------------------------------
// LOOP
// ---------------------------------------------------------

void loop() {
  bool connected = keyboard.isConnected();

  // Okresowy pomiar baterii
  if (millis() - lastBatteryCheckTime >= BATTERY_CHECK_INTERVAL_MS) {
    lastBatteryCheckTime = millis();
    updateBatteryState();
  }

  handleLowBatteryLed();

  bool currentPlay = digitalRead(PLAY_PIN);
  bool currentForward = digitalRead(FORWARD_PIN);
  bool currentPrev = digitalRead(PREVIOUS_PIN);

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {

    if (connected) {

      if (currentPlay == LOW && lastPlayState == HIGH) {
        sendCommandForButton(PLAY_PIN);
        lastDebounceTime = millis();
      }

      if (currentForward == LOW && lastForwardState == HIGH) {
        sendCommandForButton(FORWARD_PIN);
        lastDebounceTime = millis();
      }

      if (currentPrev == LOW && lastPrevState == HIGH) {
        sendCommandForButton(PREVIOUS_PIN);
        lastDebounceTime = millis();
      }
    }

    if (anyButtonPressed()) {
      lastActivityTime = millis();
    }
  }

  lastPlayState = currentPlay;
  lastForwardState = currentForward;
  lastPrevState = currentPrev;

  // WAŻNE:
  // Gdy BLE jest połączone, NIE robimy ręcznego light-sleep.
  // Dzięki temu połączenie Bluetooth nie powinno się zrywać.

  if (!connected && !anyButtonPressed()) {
    if (millis() - bootTime > NO_CONNECTION_SLEEP_MS) {
      goToLightSleepWhenDisconnected();
    }
  }

  delay(10);
}