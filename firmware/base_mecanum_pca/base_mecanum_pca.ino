// base_mecanum_pca — 메카넘 4WD 최종 명령 펌웨어
//   UNO + PCA9685(I2C 0x60) + MX1508 H-브리지 ×4
//
// PCA9685 채널을 직접(raw) 제어. 각 모터 = 2채널(정방향/역방향 입력).
// 채널맵은 base_pca_scan 으로 실측한 값 (2026-06-02):
//   FL 왼앞  정8 역9     FR 오앞  정12 역13
//   RL 왼뒤  정10 역11    RR 오뒤  정14 역15
//
// 프로토콜 (memory: project-serial-protocols):
//   수신: <BASE,fl,fr,rl,rr>\n   휠 선속도 m/s, 부호 포함 (+ = 전진방향)
//   송신: <HB,fl,fr,rl,rr>\n     5Hz, 현재 적용 명령 echo (엔코더 없음 → 실측 아님)
//
// 안전: 워치독 500ms 무명령→정지, 부팅 시 전 채널 0.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);

// 휠 순서 0=FL 1=FR 2=RL 3=RR
const uint8_t FWD_CH[4] = { 8, 12, 10, 14 };  // FL, FR, RL, RR 정방향
const uint8_t REV_CH[4] = { 9, 13, 11, 15 };  //                역방향

const float    MAX_SPEED = 0.6;    // m/s 풀스로틀 대응 (캘리브레이션 전 임시 스케일)
const uint16_t PCA_MIN   = 1500;   // 정지마찰 극복 최소 duty(12bit) — 실측 2500 구동 확인됨
const uint16_t PCA_MAX   = 4095;
const float    DEADBAND  = 0.02;
const unsigned long WATCHDOG_MS = 500;

float wheelCmd[4] = { 0, 0, 0, 0 };
unsigned long lastCmdMs = 0, lastHb = 0;
char buf[80];
uint8_t idx = 0;
bool inFrame = false;
const int LED = 13;

void setWheelPWM(uint8_t w, float v) {
  float mag = fabs(v) / MAX_SPEED;
  if (mag > 1.0) mag = 1.0;
  uint8_t fc = FWD_CH[w], rc = REV_CH[w];
  if (mag < DEADBAND) {
    pwm.setPWM(fc, 0, 0);
    pwm.setPWM(rc, 0, 0);
    return;
  }
  uint16_t duty = PCA_MIN + (uint16_t)((float)(PCA_MAX - PCA_MIN) * mag);
  if (v >= 0) {                 // 전진
    pwm.setPWM(rc, 0, 0);
    pwm.setPWM(fc, 0, duty);
  } else {                      // 후진
    pwm.setPWM(fc, 0, 0);
    pwm.setPWM(rc, 0, duty);
  }
}

void stopAll() {
  for (uint8_t w = 0; w < 4; w++) {
    wheelCmd[w] = 0;
    pwm.setPWM(FWD_CH[w], 0, 0);
    pwm.setPWM(REV_CH[w], 0, 0);
  }
}

void parseFrame() {
  buf[idx] = '\0';
  if (strncmp(buf, "BASE,", 5) != 0) return;
  float v[4];
  char* p = buf + 5;
  for (uint8_t i = 0; i < 4; i++) {
    v[i] = atof(p);
    char* c = strchr(p, ',');
    if (i < 3) {
      if (!c) return;
      p = c + 1;
    }
  }
  for (uint8_t i = 0; i < 4; i++) wheelCmd[i] = v[i];
  lastCmdMs = millis();
  for (uint8_t w = 0; w < 4; w++) setWheelPWM(w, wheelCmd[w]);
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(1600);
  stopAll();
  delay(200);
  Serial.println("<BANNER,base_mecanum_pca_v1,0x60,115200>");
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
  if (now - lastHb >= 200) {
    lastHb = now;
    digitalWrite(LED, !digitalRead(LED));
    Serial.print("<HB,");
    Serial.print(wheelCmd[0], 3); Serial.print(',');
    Serial.print(wheelCmd[1], 3); Serial.print(',');
    Serial.print(wheelCmd[2], 3); Serial.print(',');
    Serial.print(wheelCmd[3], 3);
    Serial.println(">");
  }
}
