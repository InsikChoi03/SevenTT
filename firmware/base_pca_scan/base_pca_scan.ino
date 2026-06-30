// base_pca_scan — PCA9685(0x60) 채널 단독 구동 도구. MX1508 채널↔모터 매핑 실측용.
//
// MX1508은 모터당 2입력(IN1/IN2). PCA9685 한 채널만 PWM 주고 나머지 0이면
// 그 채널이 어떤 모터의 한쪽 입력이면 그 모터가 한 방향으로 돈다.
// → 채널 0~15를 하나씩 때려보면 "채널 = 모터 + 방향"을 전부 알아낼 수 있다.
//
// 명령(시리얼):
//   <ONLY,ch,duty>  : 전 채널 0 으로 끄고 ch 만 duty(0~4095)로  ← 매핑용 핵심
//   <CH,ch,duty>    : ch 만 duty 로 (나머진 그대로)
//   <OFF>           : 전 채널 0
// 안전: 2초간 새 ONLY/CH 없으면 전 채널 자동 OFF (호스트 죽어도 정지).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);

char buf[80];
uint8_t idx = 0;
bool inFrame = false;

unsigned long lastCmdMs = 0;
bool active = false;
unsigned long lastHb = 0;
const int LED = 13;
const unsigned long WATCHDOG_MS = 2000;

void allOff() {
  for (uint8_t c = 0; c < 16; c++) pwm.setPWM(c, 0, 0);
  active = false;
}

void setChan(int ch, long duty) {
  if (ch < 0 || ch > 15) return;
  if (duty < 0) duty = 0;
  if (duty > 4095) duty = 4095;
  pwm.setPWM(ch, 0, (uint16_t)duty);
  if (duty > 0) { active = true; lastCmdMs = millis(); }
}

void parseFrame() {
  buf[idx] = '\0';
  if (!strncmp(buf, "ONLY,", 5)) {
    char* p = buf + 5;
    int ch = atoi(p);
    char* c = strchr(p, ',');
    long duty = c ? atol(c + 1) : 2500;
    for (uint8_t k = 0; k < 16; k++) pwm.setPWM(k, 0, 0);
    setChan(ch, duty);
    Serial.print("<ONLY,"); Serial.print(ch); Serial.print(','); Serial.print(duty); Serial.println(">");
  } else if (!strncmp(buf, "CH,", 3)) {
    char* p = buf + 3;
    int ch = atoi(p);
    char* c = strchr(p, ',');
    long duty = c ? atol(c + 1) : 0;
    setChan(ch, duty);
    Serial.print("<CH,"); Serial.print(ch); Serial.print(','); Serial.print(duty); Serial.println(">");
  } else if (!strncmp(buf, "OFF", 3)) {
    allOff();
    Serial.println("<OFF>");
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(1600);
  allOff();
  delay(200);
  Serial.println("<BANNER,base_pca_scan_v1,0x60>");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') { inFrame = true; idx = 0; }
    else if (c == '>' && inFrame) { inFrame = false; parseFrame(); }
    else if (inFrame && idx < sizeof(buf) - 1) { buf[idx++] = c; }
  }
  unsigned long now = millis();
  if (active && now - lastCmdMs > WATCHDOG_MS) allOff();
  if (now - lastHb >= 500) { lastHb = now; digitalWrite(LED, !digitalRead(LED)); }
}
