// base_arm_combined — 통합 펌웨어: 메카넘 베이스(<BASE>@PCA9685 0x60) + 2R 팔(<ARM>@PCA9685 0x40)
//   한 아두이노(UNO/CH340)에서 둘 다 제어. 2026-07-02 팔 재구성(어깨/손목/그리퍼 3서보) 반영.
//   base_mecanum_pca.ino + arm_servo.ino 병합. 두 펌웨어의 검증값 그대로.
//
// 프로토콜:
//   수신 <BASE,fl,fr,rl,rr>\n   휠 선속도 m/s (부호=방향). base_mecanum_pca와 동일
//   수신 <ARM,t1,t2,t3,...>\n   서보각 0~180: t1=ch0 어깨, t2=ch1 손목, t3=ch2 그리퍼 (나머지 무시)
//   송신 <HB,fl,fr,rl,rr>\n     5Hz 베이스 하트비트(명령 echo, 엔코더 없음)
//   송신 <ODOM,fl,fr,rl,rr,t_ms>\n   10Hz 엔코더 실측 휠 속도(m/s)
//   송신 <ENC,e1,e2,e3,e4,t_ms>\n       2Hz 엔코더 raw tick(매핑 검증용)
//   송신 <ARMACK,a,b,c>\n       <ARM> 수신 echo (최대 5Hz)
//   송신 <BANNER,...>           부팅 시 1회
//
// PCA9685 둘(같은 I2C 버스, 주소 다름): 베이스 0x60@1600Hz(MX1508 H-브리지), 팔 0x40@50Hz(서보 500~2500us).
// 안전: 베이스 워치독 500ms 무명령→정지, 부팅 시 휠 정지.
//   팔은 **부팅 시 limp(무구동)** — 그리퍼/자세 안전각 미확정이라 스냅 방지. 첫 <ARM> 수신부터 구동(그 값으로 스냅→이후 smooth).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/interrupt.h>

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

// ---- 엔코더 (QGPMaker V5.2 기본 핀맵) ----
// Encoder1=(8,9), Encoder2=(6,7), Encoder3=(3,2), Encoder4=(5,4).
// 실측 매핑:
//   FL->E1(신호 약함, 임시 제외 가능), FR->E4, RL->E2, RR->E3
// 부호는 RL만 반전이 필요했다.
const uint8_t ENC_A[4] = { 8, 6, 3, 5 };
const uint8_t ENC_B[4] = { 9, 7, 2, 4 };
const uint8_t WHEEL_ENCODER[4] = { 0, 3, 1, 2 };  // FL,FR,RL,RR 각각이 쓰는 Encoder index(0=E1)
const int8_t WHEEL_ENC_DIR[4] = { +1, +1, -1, +1 };
const long ENC_COUNTS_PER_REV = 4320;             // PPR 12 * x4 * gear 90 (판매처 RPM 예제 기준)
const float WHEEL_DIAMETER_M = 0.065;             // TODO: 실측 후 보정
const float WHEEL_CIRC_M = WHEEL_DIAMETER_M * 3.1415926;
const unsigned long ODOM_MS = 100;                // 10Hz
const unsigned long ENC_DEBUG_MS = 500;           // 2Hz

volatile int32_t encTicks[4] = { 0, 0, 0, 0 };
volatile uint8_t encState[4] = { 0, 0, 0, 0 };
int32_t lastOdomTicks[4] = { 0, 0, 0, 0 };
unsigned long lastOdom = 0, lastEncDebug = 0;

// ---- 전방 초음파 HC-SR04 (echo=D8, trig=D9) ----
// 구 엔코더 E1(FL) 핀 재활용: FL 엔코더는 신호가 약해 원래도 제외 가능(위 주석). E1은 setupEncoders에서 건너뜀.
const uint8_t SR04_ECHO = 8;
const uint8_t SR04_TRIG = 9;
const unsigned long SR04_MS = 66;                 // ~15Hz
const unsigned long SR04_TIMEOUT_US = 12000UL;    // ~2 m 최대(벽 감지엔 충분, 블로킹 최소화)
unsigned long lastSr04 = 0;

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
// D13 is wired to the lift MOSFET, so do not use it as a heartbeat LED.
const int LED = -1;

// ---- 광각 리프트 모터 MOSFET (D13, on/off 스위치) ----
// D13 is not used by the encoder pin map below; keep LED heartbeat disabled while lift is on D13.
const int LIFT_PIN = 13;
const unsigned long MAX_LIFT_MS = 10000;   // 스톨 번아웃 방지: 최대 on 시간 후 자동 off
bool liftOn = false;
unsigned long liftOffAt = 0;

void updateEncoder(uint8_t e) {
  uint8_t state = encState[e] & 3;
  if (digitalRead(ENC_A[e])) state |= 4;
  if (digitalRead(ENC_B[e])) state |= 8;
  encState[e] = state >> 2;
  switch (state) {
    case 1:
    case 7:
    case 8:
    case 14:
      encTicks[e]++;
      break;
    case 2:
    case 4:
    case 11:
    case 13:
      encTicks[e]--;
      break;
    case 3:
    case 12:
      encTicks[e] += 2;
      break;
    case 6:
    case 9:
      encTicks[e] -= 2;
      break;
  }
}

ISR(PCINT0_vect) {  // D8,D9 -> Encoder1
  updateEncoder(0);
}

ISR(PCINT2_vect) {  // D2..D7 -> Encoder2~4
  updateEncoder(1);
  updateEncoder(2);
  updateEncoder(3);
}

void enablePinChangeInterrupt(uint8_t pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

void setupEncoders() {
  for (uint8_t e = 0; e < 4; e++) {
    // E1 pins (D8/D9) are now the HC-SR04 -> skip so its PCINT doesn't fight pulseIn.
    if (ENC_A[e] == SR04_ECHO || ENC_B[e] == SR04_TRIG) continue;
    pinMode(ENC_A[e], INPUT_PULLUP);
    pinMode(ENC_B[e], INPUT_PULLUP);
    encState[e] = (digitalRead(ENC_A[e]) ? 1 : 0) | (digitalRead(ENC_B[e]) ? 2 : 0);
    enablePinChangeInterrupt(ENC_A[e]);
    enablePinChangeInterrupt(ENC_B[e]);
  }
}

void publishUltrasonic(unsigned long now) {
  if (now - lastSr04 < SR04_MS) return;
  lastSr04 = now;
  digitalWrite(SR04_TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(SR04_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(SR04_TRIG, LOW);
  unsigned long dur = pulseIn(SR04_ECHO, HIGH, SR04_TIMEOUT_US);  // blocks up to timeout
  int cm = (dur == 0) ? -1 : (int)(dur / 58);                     // -1 = no echo / beyond range
  Serial.print("<US,"); Serial.print(cm); Serial.println(">");
}

void copyEncoderTicks(int32_t out[4]) {
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) out[i] = encTicks[i];
  interrupts();
}

void resetEncoders() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) encTicks[i] = 0;
  interrupts();
  for (uint8_t i = 0; i < 4; i++) lastOdomTicks[i] = 0;
  lastOdom = millis();
}

void publishEncoderOdometry(unsigned long now) {
  if (lastOdom == 0) {
    resetEncoders();
    return;
  }
  unsigned long dtMs = now - lastOdom;
  if (dtMs < ODOM_MS) return;
  if (dtMs == 0) return;

  int32_t ticks[4];
  int32_t delta[4];
  float encSpeed[4];
  float wheelSpeed[4];
  copyEncoderTicks(ticks);

  for (uint8_t e = 0; e < 4; e++) {
    delta[e] = ticks[e] - lastOdomTicks[e];
    lastOdomTicks[e] = ticks[e];
    encSpeed[e] = ((float)delta[e] * 1000.0 / (float)dtMs) * WHEEL_CIRC_M / (float)ENC_COUNTS_PER_REV;
  }
  lastOdom = now;

  for (uint8_t w = 0; w < 4; w++) {
    uint8_t e = WHEEL_ENCODER[w];
    wheelSpeed[w] = (float)WHEEL_ENC_DIR[w] * encSpeed[e];
  }

  Serial.print("<ODOM,");
  Serial.print(wheelSpeed[0], 4); Serial.print(',');
  Serial.print(wheelSpeed[1], 4); Serial.print(',');
  Serial.print(wheelSpeed[2], 4); Serial.print(',');
  Serial.print(wheelSpeed[3], 4); Serial.print(',');
  Serial.print(now);
  Serial.println(">");

  if (now - lastEncDebug >= ENC_DEBUG_MS) {
    lastEncDebug = now;
    Serial.print("<ENC,");
    Serial.print(ticks[0]); Serial.print(',');
    Serial.print(ticks[1]); Serial.print(',');
    Serial.print(ticks[2]); Serial.print(',');
    Serial.print(ticks[3]); Serial.print(',');
    Serial.print(now);
    Serial.println(">");
  }
}

void setLift(bool on) {
  if (LIFT_PIN < 0) return;
  digitalWrite(LIFT_PIN, on ? HIGH : LOW);
}

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
  } else if (strncmp(buf, "LIFT,", 5) == 0) {   // 광각 리프트 모터 MOSFET (D13)
    long ms = atol(buf + 5);
    if (ms <= 0) {                               // <LIFT,0> = 즉시 off
      setLift(false); liftOn = false;
    } else {                                     // <LIFT,ms> = ms 동안 on 후 자동 off
      if (ms > (long)MAX_LIFT_MS) ms = MAX_LIFT_MS;
      if (LIFT_PIN >= 0) {
        setLift(true); liftOn = true; liftOffAt = millis() + ms;
      } else {
        liftOn = false;
      }
    }
    Serial.print("<LIFTACK,"); Serial.print(liftOn ? ms : 0); Serial.println(">");
  } else if (strncmp(buf, "ENCZERO", 7) == 0) {
    resetEncoders();
    Serial.println("<ENCZERO,OK>");
  }
}

void setup() {
  if (LED >= 0) pinMode(LED, OUTPUT);
  if (LIFT_PIN >= 0) { pinMode(LIFT_PIN, OUTPUT); setLift(false); }   // 부팅 시 리프트 모터 off
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(3000, true);          // I2C 행 방지 (서보 노이즈 대비)
  setupEncoders();
  pinMode(SR04_TRIG, OUTPUT); digitalWrite(SR04_TRIG, LOW);   // 전방 초음파
  pinMode(SR04_ECHO, INPUT);
  delay(50);
  scanI2C();                                // 부팅 시 I2C 스캔 리포트 (0x40 팔 / 0x60 베이스 감지)
  pwmBase.begin(); pwmBase.setPWMFreq(1600); // 휠 PWM (MX1508)
  pwmArm.begin();  pwmArm.setOscillatorFrequency(27000000); pwmArm.setPWMFreq(50);  // 서보
  delay(10);
  stopAll();                                 // 휠 정지
  for (int i = 0; i < 3; i++) { curA[i] = 90; tgtA[i] = 90; }   // 팔은 boot-limp (아래 loop서 armActive 전엔 미구동)
  delay(200);
  resetEncoders();
  Serial.println("<BANNER,base_arm_combined_v2,base0x60/arm0x40,enc4320,115200>");
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
    setLift(false); liftOn = false;
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

  publishEncoderOdometry(now);
  publishUltrasonic(now);       // 전방 초음파 <US,cm> ~15Hz

  // 베이스 HB 5Hz
  if (now - lastHb >= 200) {
    lastHb = now;
    if (LED >= 0) digitalWrite(LED, !digitalRead(LED));
    Serial.print("<HB,");
    Serial.print(wheelCmd[0], 3); Serial.print(',');
    Serial.print(wheelCmd[1], 3); Serial.print(',');
    Serial.print(wheelCmd[2], 3); Serial.print(',');
    Serial.print(wheelCmd[3], 3);
    Serial.println(">");
  }
}
