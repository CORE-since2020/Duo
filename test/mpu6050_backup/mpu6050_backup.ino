#include "mpu6050.h"


mpu6050 mpu(21,22,1,1);

void setup() {
  //mpu6050の初期化
  mpu.setup();
  //シリアル通信の初期化
  Serial.begin(115200);
}

void loop() {
  int16_t data[3];
  int16_t g_data[3];
  double acc_x, acc_y, acc_z;
  double gyro_x, gyro_y, gyro_z;

  mpu.getValueRawACC(data);
  mpu.getValueRawGYRO(g_data);

  acc_x = data[0] / 8192.0;
  acc_y = data[1] / 8192.0;
  acc_z = data[2] / 8192.0;

  gyro_x = g_data[0] / 65.5;
  gyro_y = g_data[1] / 65.5;
  gyro_z = g_data[2] / 65.5;

  Serial.print("X："); Serial.print(acc_x);  Serial.print("[G] ");
  Serial.print("Y："); Serial.print(acc_y);  Serial.print("[G] ");
  Serial.print("Z："); Serial.print(acc_z);  Serial.print("[G] ");

  Serial.print("X："); Serial.print(gyro_x);  Serial.print("[deg/s] ");
  Serial.print("Y："); Serial.print(gyro_y);  Serial.print("[deg/s] ");
  Serial.print("Z："); Serial.print(gyro_z);  Serial.println("[deg/s]");
  
  delay(100);
}
