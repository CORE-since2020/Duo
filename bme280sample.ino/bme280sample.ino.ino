/* BME280(気圧・気温・湿度センサ)のテストコード */

/* BME280初期設定 */

 uint8_t osrs_t   = 1;      //Temperature oversampling x 1
  uint8_t osrs_p   = 1;      //Pressure oversampling    x 1
  uint8_t osrs_h   = 1;      //Humidity oversampling    x 1
  uint8_t mode     = 3;      //Normal mode
  uint8_t t_sb     = 0;      //Tstandby 0.5ms
  uint8_t filter   = 4;      //IIR Filter x1
  uint8_t spi3w_en = 0;      //3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;     //温度、圧力、オーバーサンプリング
  uint8_t config_reg    = (t_sb << 5)   | (filter << 2) | spi3w_en; //スタンバイ時間とIIRフィルター設定
  uint8_t ctrl_hum_reg  =  osrs_h;                                  //湿度オーバーサンプリング


/* 設定値を送信 */

Wire.beginTransmission(0x76);
  Wire.write(0xF2);
  Wire.write(ctrl_hum_reg);
  Wire.endTransmission();

  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  Wire.write(ctrl_meas_reg);
  Wire.endTransmission();

  Wire.beginTransmission(0x76);
  Wire.write(0xF5);
  Wire.write(config_reg);
  Wire.endTransmission();


/* パラメータの読み出し */

int8_t   dig_H6;
uint8_t  dig_H1, dig_H3;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int16_t  dig_H2, dig_H4, dig_H5;
uint16_t dig_T1, dig_P1;

Wire.beginTransmission(0x76);
Wire.write(0x88);
Wire.endTransmission();
Wire.requestFrom(0x76, 24);

byte inBuf[32], i = 0;
while (Wire.available()) {
  inBuf[i] = Wire.read();
  i++;
}

Wire.beginTransmission(0x76);
Wire.write(0xA1);
Wire.endTransmission();
Wire.requestFrom(0x76, 1);

while (Wire.available() < 1);
inBuf[i] = Wire.read();
i++;

Wire.beginTransmission(0x76);
Wire.write(0xE1);
Wire.endTransmission();
Wire.requestFrom(0x76, 7);

while (Wire.available()) {
  inBuf[i] = Wire.read();
  i++;
}

dig_T1 = (inBuf[1] << 8)  | inBuf[0];
dig_T2 = (inBuf[3] << 8)  | inBuf[2];
dig_T3 = (inBuf[5] << 8)  | inBuf[4];
dig_P1 = (inBuf[7] << 8)  | inBuf[6];
dig_P2 = (inBuf[9] << 8)  | inBuf[8];
dig_P3 = (inBuf[11] << 8) | inBuf[10];
dig_P4 = (inBuf[13] << 8) | inBuf[12];
dig_P5 = (inBuf[15] << 8) | inBuf[14];
dig_P6 = (inBuf[17] << 8) | inBuf[16];
dig_P7 = (inBuf[19] << 8) | inBuf[18];
dig_P8 = (inBuf[21] << 8) | inBuf[20];
dig_P9 = (inBuf[23] << 8) | inBuf[22];
dig_H1 =  inBuf[24];
dig_H2 = (inBuf[26] << 8) | inBuf[25];
dig_H3 =  inBuf[27];
dig_H4 = (inBuf[28] << 4) | (0x0F & inBuf[29]);
dig_H5 = (inBuf[30] << 4) | ((inBuf[29] >> 4) & 0x0F);
dig_H6 =  inBuf[31];


/* 気圧・温度・湿度のデータの読み取り */

  int i = 0;
  uint32_t data[8];
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 8);

  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  presRaw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  tempRaw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  humRaw  = (data[6] << 8) | data[7];


/* 気圧計算 */

inline uint32_t computePres32(int32_t adc_P) {
  int32_t var1, var2;
  uint32_t P;
  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
  var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
  if (var1 == 0) {
    return 0;
  }
  P = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000) {
    P = (P << 1) / ((uint32_t) var1);
  }
  else {
    P = (P / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)dig_P9) * ((int32_t)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(P >> 2)) * ((int32_t)dig_P8)) >> 13;
  P = (uint32_t)((int32_t)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}


/* 温度計算 */

inline int32_t computeTemp32(int32_t adc_T) {
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}


/* 湿度計算 */

uint32_t computeHum32(int32_t adc_H) {
  int32_t v_x1;
  v_x1 = (t_fine - ((int32_t)76800));
  v_x1 = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1)) +
            ((int32_t)16384)) >> 15) * (((((((v_x1 * ((int32_t)dig_H6)) >> 10) *
                                          (((v_x1 * ((int32_t)dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + (( int32_t)2097152)) *
                                         ((int32_t) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (uint32_t)(v_x1 >> 12);
}
