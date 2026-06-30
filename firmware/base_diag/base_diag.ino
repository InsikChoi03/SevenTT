// base_diag — 시리얼→MCU 경로 검증용 진단 스케치 (모터/12V 불필요)
//
// 목적: Jetson → ch341 → UNO RX → 스케치 파싱 경로가 실제로 살아있는지,
//       12V/모터 없이 보드 LED(13)와 시리얼 echo만으로 확인한다.
//
// 동작:
//   - 부팅 시   : <BANNER,base_diag_v1,115200>
//   - 1초마다   : <HB,n> 하트비트 + LED 토글 (느린 깜빡)
//   - <BASE,..> 수신 시 : 즉시 <RX,...> echo + LED 빠르게 3회 (눈으로도 확인)
//
// 호스트 확인:
//   python3 scripts/base_serial_test.py --port /dev/ttyUSB0 --no-jog --listen 5
//   → <BANNER...> 와 <HB,..> 가 보이면 경로 정상.
//   → jog 시 <RX,BASE,...> echo 가 돌아오면 명령 수신까지 정상.

const int LED = 13;

char buf[64];
byte idx = 0;
bool inFrame = false;

unsigned long lastHB = 0;
unsigned int hb = 0;

void blinkFast(int n, int ms) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, HIGH);
    delay(ms);
    digitalWrite(LED, LOW);
    delay(ms);
  }
}

void handleFrame() {
  buf[idx] = '\0';
  Serial.print("<RX,");
  Serial.print(buf);     // '<' 와 '>' 사이 내용 그대로 echo
  Serial.println(">");
  blinkFast(3, 40);
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  delay(200);
  Serial.println("<BANNER,base_diag_v1,115200>");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') {
      inFrame = true;
      idx = 0;
    } else if (c == '>' && inFrame) {
      inFrame = false;
      handleFrame();
    } else if (inFrame && idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }

  unsigned long now = millis();
  if (now - lastHB >= 1000) {
    lastHB = now;
    digitalWrite(LED, !digitalRead(LED));
    Serial.print("<HB,");
    Serial.print(hb++);
    Serial.println(">");
  }
}
