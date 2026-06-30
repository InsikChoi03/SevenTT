// base_pca9685 — 메카넘 4WD 명령 펌웨어 (UNO + PCA9685@0x60 + MX1508 H-브리지 ×4)
//
// 이 베이스 보드는 L293D가 아니라 PCA9685(I2C 0x60)가 MX1508 H-브리지들을 PWM 구동.
// Adafruit Motor Shield V2 호환(주소 0x60)이라 가정하고 그 라이브러리로 M1~M4 제어.
// → 만약 이 펌웨어로도 안 움직이면 채널 매핑이 비표준 → base_pca_scan 으로 실측.
//
// 프로토콜 (memory: project-serial-protocols):
//   수신: <BASE,fl,fr,rl,rr>\n   휠 선속도 m/s, 부호 포함
//   송신: <HB,fl,fr,rl,rr>\n     5Hz 하트비트 = 현재 적용 명령 echo (엔코더 없음→실측 아님)
//
// 안전: 워치독 500ms 무명령→정지, 부팅 시 전 모터 RELEASE.
//
// 매핑 (jog로 실측 후 조정):
//   휠 0=FL 1=FR 2=RL 3=RR ; WHEEL2M = 각 휠이 붙은 모터단자 M1~M4 ; WHEEL_DIR 전진부호.

#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);
Adafruit_DCMotor* mtr[4];   // mtr[0..3] = M1..M4

const uint8_t WHEEL2M[4]   = { 1, 2, 3, 4 };       // FL,FR,RL,RR -> M1..M4 (실측 조정)
const int8_t  WHEEL_DIR[4] = { +1, +1, +1, +1 };   // 전진 부호 (거꾸로면 -1)

const float        MAX_SPEED   = 0.6;
const uint8_t      PWM_MIN     = 60;
const uint8_t      PWM_MAX     = 255;
const unsigned long WATCHDOG_MS = 500;

float wheelCmd[4] = { 0, 0, 0, 0 };
unsigned long lastCmdMs = 0;
unsigned long lastHbMs  = 0;

char buf[80];
uint8_t idx = 0;
bool inFrame = false;

const int LED = 13;

void applyWheel(uint8_t w) {
  float v = wheelCmd[w] * (float)WHEEL_DIR[w];
  Adafruit_DCMotor* m = mtr[WHEEL2M[w] - 1];
  float mag = fabs(v) / MAX_SPEED;
  if (mag > 1.0) mag = 1.0;
  if (mag < 0.02) {
    m->setSpeed(0);
    m->run(RELEASE);
    return;
  }
  uint8_t pwm = PWM_MIN + (uint8_t)((float)(PWM_MAX - PWM_MIN) * mag);
  m->setSpeed(pwm);
  m->run(v >= 0 ? FORWARD : BACKWARD);
}

void stopAll() {
  for (uint8_t w = 0; w < 4; w++) {
    wheelCmd[w] = 0;
    mtr[w]->setSpeed(0);
    mtr[w]->run(RELEASE);
  }
}

void parseFrame() {
  buf[idx] = '\0';
  if (strncmp(buf, "BASE,", 5) != 0) return;
  float v[4];
  char* p = buf + 5;
  for (uint8_t i = 0; i < 4; i++) {
    v[i] = atof(p);
    char* comma = strchr(p, ',');
    if (i < 3) {
      if (!comma) return;
      p = comma + 1;
    }
  }
  for (uint8_t i = 0; i < 4; i++) wheelCmd[i] = v[i];
  lastCmdMs = millis();
  for (uint8_t w = 0; w < 4; w++) applyWheel(w);
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  AFMS.begin();                 // 기본 1.6kHz PWM
  for (uint8_t i = 0; i < 4; i++) mtr[i] = AFMS.getMotor(i + 1);
  stopAll();
  delay(200);
  Serial.println("<BANNER,base_pca9685_v1,0x60,115200>");
  lastCmdMs = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') { inFrame = true; idx = 0; }
    else if (c == '>' && inFrame) { inFrame = false; parseFrame(); }
    else if (inFrame && idx < sizeof(buf) - 1) { buf[idx++] = c; }
  }

  unsigned long now = millis();

  if (now - lastCmdMs > WATCHDOG_MS) {
    bool moving = false;
    for (uint8_t w = 0; w < 4; w++) if (wheelCmd[w] != 0) moving = true;
    if (moving) stopAll();
  }

  if (now - lastHbMs >= 200) {
    lastHbMs = now;
    digitalWrite(LED, !digitalRead(LED));
    Serial.print("<HB,");
    Serial.print(wheelCmd[0], 3); Serial.print(',');
    Serial.print(wheelCmd[1], 3); Serial.print(',');
    Serial.print(wheelCmd[2], 3); Serial.print(',');
    Serial.print(wheelCmd[3], 3);
    Serial.println(">");
  }
}
