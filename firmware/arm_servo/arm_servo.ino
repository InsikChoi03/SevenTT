// arm_servo v2 — Scipia A2T 6서보 명령 펌웨어 (UNO + PCA9685@0x40) + 속도제한 보간(smooth)
//
// 프로토콜 (project-serial-protocols / mcu_bridge_arm_node):
//   수신: <ARM,t1,t2,t3,t4,t5,t6>\n   서보각 0~180 (정수)  ← 목표(target)로 설정
//         t1 base, t2 shoulder, t3 elbow, t4 wristPitch, t5 wristRoll, t6 gripper
//   송신: <ARMACK,t1..t6>\n   목표값 echo
//
// 보간: 매 20ms 각 서보를 목표로 최대 MAX_STEP°씩 이동 → 부드럽게 연속. (점프 X)
// 부팅 시 HOME {70,96,55,55,90,90}. 위치 유지(릴리스 없음). 500~2500us=0~180, 50Hz.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const int US_MIN = 500;
const int US_MAX = 2500;
const int SERVO_FREQ = 50;
const float HOME[6] = { 70, 96, 55, 55, 90, 5 };   // ch5 집게 90→5(열림): 새 CLOSED=70 초과 과부하 방지. ⚠️재플래시 필요
const float MAX_STEP = 2.0;          // deg per update(30ms) → ~67 deg/s (속도제한; 2026-06-24 3.0→2.0 하향: 픽 하강 부드럽게/박힘 완화)
const unsigned long UPDATE_MS = 30;  // I2C 트래픽 ↓ (50Hz→33Hz)

float cur[6];   // 현재 적용각
float tgt[6];   // 목표각
int lastWritten[6] = { -1, -1, -1, -1, -1, -1 };  // 변한 채널만 I2C 쓰기용
unsigned long lastUpd = 0, lastHb = 0, lastAck = 0;
char buf[80];
uint8_t idx = 0;
bool inFrame = false;
const int LED = 13;
const int RESET_SIG = 9;   // 리셋 시 신호 출력용 (비어있는 디지털 핀)

// 리셋(부팅) 시: HIGH 1.5초 → (LOW 0.1초 / HIGH 0.1초) 3번 반복 → LOW(대기)
void resetSignal() {
  pinMode(RESET_SIG, OUTPUT);
  digitalWrite(RESET_SIG, HIGH);
  delay(1500);
  for (int i = 0; i < 3; i++) {
    digitalWrite(RESET_SIG, LOW);
    delay(100);
    digitalWrite(RESET_SIG, HIGH);
    delay(100);
  }
  digitalWrite(RESET_SIG, LOW);   // 대기 상태(끔). 켠 채로 두려면 이 줄 삭제
}

void writeServo(int ch, float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  pwm.writeMicroseconds(ch, map((long)(deg * 10), 0, 1800, US_MIN, US_MAX));
}

void parseFrame() {
  buf[idx] = '\0';
  if (strncmp(buf, "ARM,", 4) != 0) return;
  float v[6];
  char* p = buf + 4;
  for (int i = 0; i < 6; i++) {
    v[i] = atof(p);
    char* c = strchr(p, ',');
    if (i < 5) { if (!c) return; p = c + 1; }
  }
  for (int i = 0; i < 6; i++) {
    if (v[i] < 0) v[i] = 0;
    if (v[i] > 180) v[i] = 180;
    tgt[i] = v[i];
  }
  if (millis() - lastAck >= 200) {   // ACK 최대 5Hz (스트리밍 시 TX 버퍼 폭주→멈춤 방지)
    lastAck = millis();
    Serial.print("<ARMACK");
    for (int i = 0; i < 6; i++) { Serial.print(','); Serial.print((int)tgt[i]); }
    Serial.println(">");
  }
}

void setup() {
  resetSignal();             // 리셋 직후 신호 출력 (서보 초기화 전)
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // I2C 행 방지: 3ms 타임아웃 + 버스 리셋 (서보 노이즈로 멈추는 것 방지)
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  for (int i = 0; i < 6; i++) { cur[i] = HOME[i]; tgt[i] = HOME[i]; writeServo(i, cur[i]); }
  Serial.println("<BANNER,arm_servo_v2,0x40,115200,smooth>");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') { inFrame = true; idx = 0; }
    else if (c == '>' && inFrame) { inFrame = false; parseFrame(); }
    else if (inFrame && idx < sizeof(buf) - 1) { buf[idx++] = c; }
  }

  unsigned long now = millis();
  if (now - lastUpd >= UPDATE_MS) {
    lastUpd = now;
    for (int i = 0; i < 6; i++) {
      float d = tgt[i] - cur[i];
      if (d > MAX_STEP) d = MAX_STEP;
      if (d < -MAX_STEP) d = -MAX_STEP;
      cur[i] += d;
      int deg = (int)(cur[i] + 0.5);
      if (deg != lastWritten[i]) {     // 변한 채널만 I2C 쓰기 (트래픽 ↓ → 글리치 확률 ↓)
        writeServo(i, cur[i]);
        lastWritten[i] = deg;
      }
    }
  }
  if (now - lastHb >= 1000) { lastHb = now; digitalWrite(LED, !digitalRead(LED)); }
}
