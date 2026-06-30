// base_mecanum — 메카넘 4WD 베이스 명령 펌웨어 (Arduino UNO + L293D 모터 실드 v1형)
//
// 프로토콜 (memory: project-serial-protocols):
//   수신: <BASE,fl,fr,rl,rr>\n   휠 선속도 m/s, 부호 포함 (소수)
//   송신: <HB,fl,fr,rl,rr>\n     0.2s 하트비트 = "현재 적용중인 명령" echo (링크/디버그용)
//        ※ 이 베이스는 엔코더가 없어 실측 ODOM이 아님. 명령값 그대로임.
//          추후 엔코더 추가 시 <ODOM,...,t_ms> 로 교체 (mcu_bridge_base_node가 기대하는 포맷).
//
// 안전:
//   - 워치독: 500ms 이상 새 <BASE> 없으면 전 모터 정지(RELEASE)
//   - 부팅 시 전 모터 RELEASE, 명령 받기 전까지 정지
//
// 브링업 매핑 (jog로 실측 후 아래 두 배열만 고치면 됨):
//   휠 순서 0=FL(앞왼) 1=FR(앞오) 2=RL(뒤왼) 3=RR(뒤오)
//   WHEEL2CH : 각 휠이 실드의 몇 번 모터 단자(M1=0 .. M4=3)에 물렸는지
//   WHEEL_DIR: 전진(+) 방향 부호. 그 휠이 거꾸로 돌면 -1 로.

#include <AFMotor.h>

AF_DCMotor m1(1), m2(2), m3(3), m4(4);
AF_DCMotor* mch[4] = { &m1, &m2, &m3, &m4 };

const uint8_t WHEEL2CH[4]  = { 0, 1, 2, 3 };       // FL->M1 FR->M2 RL->M3 RR->M4 (실측 조정)
const int8_t  WHEEL_DIR[4] = { +1, +1, +1, +1 };   // 전진 부호 (거꾸로면 -1)

const float        MAX_SPEED   = 0.6;   // 풀스로틀에 대응시킬 m/s (캘리브레이션 전 임시 스케일)
const uint8_t      PWM_MIN     = 60;    // 정지마찰 극복용 최소 PWM (이 미만이면 그냥 정지)
const uint8_t      PWM_MAX     = 255;
const unsigned long WATCHDOG_MS = 500;

float wheelCmd[4] = { 0, 0, 0, 0 };
unsigned long lastCmdMs = 0;
unsigned long lastHbMs  = 0;

char buf[80];
uint8_t idx = 0;
bool inFrame = false;

const int LED = 13;   // AFMotor v1은 13번 안 씀 → 하트비트용으로 안전

void applyWheel(uint8_t w) {
  float v = wheelCmd[w] * (float)WHEEL_DIR[w];
  uint8_t ch = WHEEL2CH[w];
  float mag = fabs(v) / MAX_SPEED;
  if (mag > 1.0) mag = 1.0;
  if (mag < 0.02) {
    mch[ch]->run(RELEASE);
    return;
  }
  uint8_t pwm = PWM_MIN + (uint8_t)((float)(PWM_MAX - PWM_MIN) * mag);
  mch[ch]->setSpeed(pwm);
  mch[ch]->run(v >= 0 ? FORWARD : BACKWARD);
}

void stopAll() {
  for (uint8_t w = 0; w < 4; w++) {
    wheelCmd[w] = 0;
    mch[w]->run(RELEASE);
  }
}

void parseFrame() {
  buf[idx] = '\0';
  if (strncmp(buf, "BASE,", 5) != 0) return;   // <ARM,..> 등은 무시
  float v[4];
  char* p = buf + 5;
  for (uint8_t i = 0; i < 4; i++) {
    v[i] = atof(p);
    char* comma = strchr(p, ',');
    if (i < 3) {
      if (!comma) return;          // 필드 부족 → 무시
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
  stopAll();
  delay(200);
  Serial.println("<BANNER,base_mecanum_v1,115200>");
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

  // 워치독: 일정시간 무명령 → 정지
  if (now - lastCmdMs > WATCHDOG_MS) {
    bool moving = false;
    for (uint8_t w = 0; w < 4; w++) if (wheelCmd[w] != 0) moving = true;
    if (moving) stopAll();
  }

  // 하트비트 5Hz: 현재 명령 echo + LED 토글
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
