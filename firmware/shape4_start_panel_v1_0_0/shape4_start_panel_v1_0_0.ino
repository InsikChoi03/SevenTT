// Shape4 start-panel standalone wiring test v1.0.0
//
// Wiring (5V common-GND LED panel):
//   D10 -- 330R -- red LED VCC
//   D11 -- 330R -- green LED VCC
//   D12 -- 330R -- yellow LED VCC
//   GND ---------- LED common GND
//   5V ----------- switch VCC
//   A1 -- 1k ----- SW (active-low/open-drain; no external pull-down)
//
// Boot: red -> green -> yellow self-test, then READY(yellow).
// Each debounced button press cycles READY -> RUNNING -> DONE -> READY.
// This sketch never drives the robot base, arm, PCA9685, or ultrasonic sensor.

const uint8_t LED_RED_PIN    = 10;
const uint8_t LED_GREEN_PIN  = 11;
const uint8_t LED_YELLOW_PIN = 12;
const uint8_t START_SW_PIN   = A1;

const unsigned long DEBOUNCE_MS = 50;

enum TestState : uint8_t {
  TEST_READY,
  TEST_RUNNING,
  TEST_DONE,
};

TestState testState = TEST_READY;
bool rawPressed = false;
bool stablePressed = false;
unsigned long rawChangedAt = 0;
uint16_t pressCount = 0;

void setLeds(bool red, bool green, bool yellow) {
  digitalWrite(LED_RED_PIN, red ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, green ? HIGH : LOW);
  digitalWrite(LED_YELLOW_PIN, yellow ? HIGH : LOW);
}

void showState(TestState state) {
  testState = state;
  if (state == TEST_READY) {
    setLeds(false, false, true);
    Serial.println("<PANEL,READY,YELLOW>");
  } else if (state == TEST_RUNNING) {
    setLeds(false, true, false);
    Serial.println("<PANEL,RUNNING,GREEN>");
  } else {
    setLeds(true, false, false);
    Serial.println("<PANEL,DONE,RED>");
  }
}

void runLedSelfTest() {
  setLeds(true, false, false);
  delay(500);
  setLeds(false, true, false);
  delay(500);
  setLeds(false, false, true);
  delay(500);
  setLeds(false, false, false);
  delay(200);
}

void setup() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(START_SW_PIN, INPUT_PULLUP);  // internal pull-up: released=HIGH, pressed=LOW

  Serial.begin(115200);
  runLedSelfTest();

  rawPressed = digitalRead(START_SW_PIN) == LOW;
  stablePressed = rawPressed;
  rawChangedAt = millis();
  showState(TEST_READY);
  Serial.println("<PANELTEST,READY,press button to cycle>");
}

void loop() {
  unsigned long now = millis();
  bool pressed = digitalRead(START_SW_PIN) == LOW;

  if (pressed != rawPressed) {
    rawPressed = pressed;
    rawChangedAt = now;
  }

  if (pressed == stablePressed || now - rawChangedAt < DEBOUNCE_MS) return;
  stablePressed = pressed;
  if (!stablePressed) return;

  pressCount++;
  Serial.print("<BUTTON,");
  Serial.print(pressCount);
  Serial.println(">");

  if (testState == TEST_READY) {
    showState(TEST_RUNNING);
  } else if (testState == TEST_RUNNING) {
    showState(TEST_DONE);
  } else {
    showState(TEST_READY);
  }
}
