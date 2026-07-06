// base_arm_combined — 통합 펌웨어: 메카넘 베이스(<BASE>@PCA9685 0x60) + 2R 팔(<ARM>@PCA9685 0x40)
//   한 아두이노(UNO/CH340)에서 둘 다 제어. 2026-07-02 팔 재구성(어깨/손목/그리퍼 3서보) 반영.
//   base_mecanum_pca.ino + arm_servo.ino 병합. 두 펌웨어의 검증값 그대로.
//
// 프로토콜:
//   수신 <BASE,fl,fr,rl,rr>\n   휠 선속도 m/s (부호=방향). base_mecanum_pca와 동일
//   수신 <ARM,t1,t2,t3,...>\n   서보각 0~180: t1=ch0 어깨, t2=ch1 손목, t3=ch2 그리퍼 (나머지 무시)
//   송신 <HB,fl,fr,rl,rr>\n     5Hz 베이스 하트비트(명령 echo, 엔코더 없음)
//   송신 <ARMACK,a,b,c>\n       <ARM> 수신 echo (최대 5Hz)
//   송신 <BANNER,...>           부팅 시 1회
//
// PCA9685 둘(같은 I2C 버스, 주소 다름): 베이스 0x60@1600Hz(MX1508 H-브리지), 팔 0x40@50Hz(서보 500~2500us).
// 안전: 베이스 워치독 500ms 무명령→정지, 부팅 시 휠 정지.
//   팔은 **부팅 시 limp(무구동)** — 그리퍼/자세 안전각 미확정이라 스냅 방지. 첫 <ARM> 수신부터 구동(그 값으로 스냅→이후 smooth).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwmBase = Adafruit_PWMServoDriver(0x60);
Adafruit_PWMServoDriver pwmArm  = Adafruit_PWMServoDriver(0x40);

// ---- 베이스 (base_mecanum_pca 값 그대로) ----
const uint8_t FWD_CH[4] = { 8, 12, 10, 14 };   // FL,FR,RL,RR 정방향
const uint8_t REV_CH[4] = { 9, 13, 11, 15 };   //           역방향
const float    MAX_SPEED = 0.6;
const uint16_t PCA_MIN   = 1500;
const uint16_t PCA_MAX   = 4095;
const float    DEADBAND  = 0.02;
const unsigned long WATCHDOG_MS = 500;
float wheelCmd[4] = { 0, 0, 0, 0 };
unsigned long lastCmdMs = 0, lastHb = 0;

// ---- 팔 (2R: ch0 어깨, ch1 손목, ch2 그리퍼) ----
const int US_MIN = 500, US_MAX = 2500;
const float MAX_STEP = 2.0;          // deg/update(30ms) ≈ 67 deg/s 속도제한
const unsigned long UPDATE_MS = 30;
float curA[3], tgtA[3];
int lastWritten[3] = { -1, -1, -1 };
bool armActive = false;              // 부팅 limp → 첫 <ARM>부터 true
unsigned long lastUpd = 0, lastAck = 0;

char buf[80];
uint8_t idx = 0;
bool inFrame = false;
const int LED = 13;

// ---- 광각 리프트 모터 MOSFET (핀 9, on/off 스위치) ----
const int LIFT_PIN = 9;
const unsigned long MAX_LIFT_MS = 10000;   // 스톨 번아웃 방지: 최대 on 시간 후 자동 off
bool liftOn = false;
unsigned long liftOffAt = 0;

void setWheelPWM(uint8_t w, float v) {
  float mag = fabs(v) / MAX_SPEED;
  if (mag > 1.0) mag = 1.0;
  uint8_t fc = FWD_CH[w], rc = REV_CH[w];
  if (mag < DEADBAND) { pwmBase.setPWM(fc, 0, 0); pwmBase.setPWM(rc, 0, 0); return; }
  uint16_t duty = PCA_MIN + (uint16_t)((float)(PCA_MAX - PCA_MIN) * mag);
  if (v >= 0) { pwmBase.setPWM(rc, 0, 0); pwmBase.setPWM(fc, 0, duty); }
  else        { pwmBase.setPWM(fc, 0, 0); pwmBase.setPWM(rc, 0, duty); }
}

void stopAll() {
  for (uint8_t w = 0; w < 4; w++) {
    wheelCmd[w] = 0;
    pwmBase.setPWM(FWD_CH[w], 0, 0);
    pwmBase.setPWM(REV_CH[w], 0, 0);
  }
}

void writeServo(int ch, float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  pwmArm.writeMicroseconds(ch, map((long)(deg * 10), 0, 1800, US_MIN, US_MAX));
}

void scanI2C() {   // 버스에 ACK하는 주소 나열 → <I2C,0x40,0x60,...> (0x40=팔, 0x60=베이스)
  Serial.print("<I2C");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(",0x");
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
    }
  }
  Serial.println(">");
}

void parseFrame() {
  buf[idx] = '\0';
  if (strncmp(buf, "BASE,", 5) == 0) {
    float v[4]; char* p = buf + 5;
    for (uint8_t i = 0; i < 4; i++) {
      v[i] = atof(p); char* c = strchr(p, ',');
      if (i < 3) { if (!c) return; p = c + 1; }
    }
    for (uint8_t i = 0; i < 4; i++) wheelCmd[i] = v[i];
    lastCmdMs = millis();
    for (uint8_t w = 0; w < 4; w++) setWheelPWM(w, wheelCmd[w]);
  } else if (strncmp(buf, "ARM,", 4) == 0) {
    float v[3]; char* p = buf + 4;
    for (uint8_t i = 0; i < 3; i++) {
      v[i] = atof(p); char* c = strchr(p, ',');
      if (i < 2) { if (!c) return; p = c + 1; }
    }
    for (uint8_t i = 0; i < 3; i++) { if (v[i] < 0) v[i] = 0; if (v[i] > 180) v[i] = 180; tgtA[i] = v[i]; }
    if (!armActive) { for (uint8_t i = 0; i < 3; i++) curA[i] = tgtA[i]; armActive = true; }  // 첫 명령=스냅
    if (millis() - lastAck >= 200) {
      lastAck = millis();
      Serial.print("<ARMACK,"); Serial.print((int)tgtA[0]); Serial.print(',');
      Serial.print((int)tgtA[1]); Serial.print(','); Serial.print((int)tgtA[2]); Serial.println(">");
    }
  } else if (strncmp(buf, "LIFT,", 5) == 0) {   // 광각 리프트 모터 MOSFET (핀9)
    long ms = atol(buf + 5);
    if (ms <= 0) {                               // <LIFT,0> = 즉시 off
      digitalWrite(LIFT_PIN, LOW); liftOn = false;
    } else {                                     // <LIFT,ms> = ms 동안 on 후 자동 off
      if (ms > (long)MAX_LIFT_MS) ms = MAX_LIFT_MS;
      digitalWrite(LIFT_PIN, HIGH); liftOn = true; liftOffAt = millis() + ms;
    }
    Serial.print("<LIFTACK,"); Serial.print(liftOn ? ms : 0); Serial.println(">");
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(LIFT_PIN, OUTPUT); digitalWrite(LIFT_PIN, LOW);   // 부팅 시 리프트 모터 off
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(3000, true);          // I2C 행 방지 (서보 노이즈 대비)
  delay(50);
  scanI2C();                                // 부팅 시 I2C 스캔 리포트 (0x40 팔 / 0x60 베이스 감지)
  pwmBase.begin(); pwmBase.setPWMFreq(1600); // 휠 PWM (MX1508)
  pwmArm.begin();  pwmArm.setOscillatorFrequency(27000000); pwmArm.setPWMFreq(50);  // 서보
  delay(10);
  stopAll();                                 // 휠 정지
  for (int i = 0; i < 3; i++) { curA[i] = 90; tgtA[i] = 90; }   // 팔은 boot-limp (아래 loop서 armActive 전엔 미구동)
  delay(200);
  Serial.println("<BANNER,base_arm_combined_v1,base0x60/arm0x40,115200>");
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

  // 광각 리프트 모터 자동 off (MAX_LIFT_MS 스톨 방지)
  if (liftOn && (long)(now - liftOffAt) >= 0) {
    digitalWrite(LIFT_PIN, LOW); liftOn = false;
    Serial.println("<LIFT,done>");
  }

  // 베이스 워치독
  if (now - lastCmdMs > WATCHDOG_MS) {
    bool moving = false;
    for (uint8_t w = 0; w < 4; w++) if (wheelCmd[w] != 0) moving = true;
    if (moving) stopAll();
  }

  // 팔 smooth 보간 (첫 <ARM> 이후에만)
  if (armActive && now - lastUpd >= UPDATE_MS) {
    lastUpd = now;
    for (int i = 0; i < 3; i++) {
      float d = tgtA[i] - curA[i];
      if (d > MAX_STEP) d = MAX_STEP;
      if (d < -MAX_STEP) d = -MAX_STEP;
      curA[i] += d;
      int deg = (int)(curA[i] + 0.5);
      if (deg != lastWritten[i]) { writeServo(i, curA[i]); lastWritten[i] = deg; }
    }
  }

  // 베이스 HB 5Hz
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
