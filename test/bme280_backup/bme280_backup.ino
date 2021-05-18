#include "bme280.h"

bme280 bme(21, 22);

void setup() {
  //bme280の初期化
  bme.setup();
  //シリアル通信の初期化
  Serial.begin(115200);

}

void loop() {
  double data[3];

  bme.getValue(data);

  Serial.print("Pressure：");  Serial.print(data[0]);  Serial.print("[hPa] ");
  Serial.print("Temperature："); Serial.print(data[1]);  Serial.print("[℃] ");
  Serial.print("Humidity：");  Serial.print(data[2]);  Serial.println("[%]");

  delay(1000);

}
