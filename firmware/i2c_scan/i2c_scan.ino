// i2c_scan — UNO의 I2C 버스(A4=SDA, A5=SCL)에 붙은 장치 주소 스캔.
// PCA9685(보통 0x40~0x7F) 가 실제로 잡히는지/주소가 뭔지 확인용.
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(300);
  Serial.println("<I2CSCAN_START>");
}

void loop() {
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("<I2C,0x");
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      Serial.println(">");
      n++;
    }
  }
  Serial.print("<I2CSCAN_DONE,");
  Serial.print(n);
  Serial.println(">");
  delay(2000);
}
