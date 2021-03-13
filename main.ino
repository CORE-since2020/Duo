//メイン電装
//加速度，ジャイロ，気圧，気温，湿度，GNSSの取得と離床判定，開放判定
//エンジン燃焼時間TBD[s]の判定どうしますか

#include <Wire.h>
#include <SD.h>
#include "TinyGPS++.h"

//フェーズ定義
enum Phaze{
  STANDBY,
  LAUNCH,
  BURNING,
  RISING,
  PARACHUTE
};

int ai; //離床判定＿移動平均カウントのための変数
int acnt = 0; //離床判定＿連続回数カウントのための変数
float asqrt[5]; //移動平均
float _asqrt[5];  


// レジスタアドレス
#define MPU6050_ACCEL_XOUT_H 0x3B //以下MPU6050 
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_I2C_ADDRESS  0x68
#define BME280_ADDR 0x76 //以下BME280
#define CONFIG 0xF5
#define CTRL_MEAS 0xF4
#define CTRL_HUM 0xF2
#define RX_PIN 16 //GPS_RX
#define TX_PIN 17 //GPS_TX
#define Wire_RX_pin 3 //無線機_RX
#define Wire_TX_pin 1 //無線機_TX
#define A_FLIGHT 2.50 //離床判定＿しきい値
const int SAMPLING_RATE = 200; //サンプリングレート200Hz

// MPU6050構造体定義
typedef union accel_t_gyro_union {
  struct {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  }
  reg;
  struct {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  }
  value;
};

//気温補正データ
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
 
//湿度補正データ
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;
 
//気圧補正データ
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

unsigned char dac[26];
unsigned int i;

int32_t t_fine;
int32_t adc_P, adc_T, adc_H;

TinyGPSPlus gps;
//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);


const char* f_name = "/main.csv"; //SD
File myFile;


//タイマ１：データ取得，タイマ２：離床判定，タイマ３：開放判定，タイマ４：データ送信
volatile int timeCounter1;
volatile int timeCounter2;
volatile int timeCounter3;
volatile int timeCounter4;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;
hw_timer_t *timer4 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//タイマ１割り込み用の関数
void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//タイマ２割り込み用の関数
void IRAM_ATTR onTimer2(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter2++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//タイマ３割り込み用の関数
void IRAM_ATTR onTimer3(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter3++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//タイマ４割り込み用の関数
void IRAM_ATTR onTimer4(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter4++;
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() {
  int error;
  uint8_t c;
  Serial.begin(115200);
  //シリアル通信が開通するまで待つ
 while (!Serial) {
 ;
 }
  Serial2.begin(9600,SERIAL_8N1,RX_PIN,TX_PIN);
  Serial2.println("Set the data rate");
  
  Wire.begin(21, 22);
  //加速度±8[G]に設定
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  //ジャイロ±2000[deg/s]に設定
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x11);
  Wire.endTransmission();

  //MPU6050初回の読み出し
  error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
  error = MPU6050_read(MPU6050_PWR_MGMT_1, &c, 1);
  MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0);

   //BME280動作設定
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();

  //BME280測定条件設定
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(CTRL_MEAS);
  Wire.write(0x24);
  Wire.endTransmission();
  
  //BME280温度測定条件設定
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(CTRL_HUM);
  Wire.write(0x01);
  Wire.endTransmission();

  //BME280補正データ取得
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(0x88);
  Wire.endTransmission();
  
  Wire.requestFrom(BME280_ADDR, 26);
  for (i=0; i<26; i++){
    while (Wire.available() == 0 ){}
    dac[i] = Wire.read();//dacにI2Cデバイス「BME280」のデータ読み込み
  }
  
  dig_T1 = ((uint16_t)((dac[1] << 8) | dac[0]));
  dig_T2 = ((int16_t)((dac[3] << 8) | dac[2]));
  dig_T3 = ((int16_t)((dac[5] << 8) | dac[4]));

  dig_P1 = ((uint16_t)((dac[7] << 8) | dac[6]));
  dig_P2 = ((int16_t)((dac[9] << 8) | dac[8]));
  dig_P3 = ((int16_t)((dac[11] << 8) | dac[10]));
  dig_P4 = ((int16_t)((dac[13] << 8) | dac[12]));
  dig_P5 = ((int16_t)((dac[15] << 8) | dac[14]));
  dig_P6 = ((int16_t)((dac[17] << 8) | dac[16]));
  dig_P7 = ((int16_t)((dac[19] << 8) | dac[18]));
  dig_P8 = ((int16_t)((dac[21] << 8) | dac[20]));
  dig_P9 = ((int16_t)((dac[23] << 8) | dac[22]));

  dig_H1 = ((uint8_t)(dac[25]));

  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0xE1);//出力データバイトを「補正データ」のアドレスに指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了
  
  Wire.requestFrom(BME280_ADDR, 7);//I2Cデバイス「BME280」に7Byteのデータ要求
  for (i=0; i<7; i++){
    while (Wire.available() == 0 ){}
    dac[i] = Wire.read();//dacにI2Cデバイス「BME280」のデータ読み込み
  }
  
  dig_H2 = ((int16_t)((dac[1] << 8) | dac[0]));
  dig_H3 = ((uint8_t)(dac[2]));
  dig_H4 = ((int16_t)((dac[3] << 4) + (dac[4] & 0x0F)));
  dig_H5 = ((int16_t)((dac[5] << 4) + ((dac[4] >> 4) & 0x0F)));
  dig_H6 = ((int8_t)dac[6]);
  
  delay(1000);//1000msec待機(1秒待機)
  

  //タイマ１割り込み初期化
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer1);

  //タイマ２割り込み初期化
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer2);

  //タイマ３割り込み初期化
  timer3 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer3, &onTimer3, true);
  timerAlarmWrite(timer3, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer3);

  //タイマ４割り込み初期化
  timer4 = timerBegin(3, 80, true);
  timerAttachInterrupt(timer4, &onTimer4, true);
  timerAlarmWrite(timer4, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer4);

  Serial.print("Phaze:STANDBY"); //フェーズ：STANDBY
  Serial.println("");
}


void loop() {
  if(/*無線でコマンド受信*/){
    Serial.print("Phaze:LAUNCH"); //フェーズ：LAUNCH
    Serial.println("");
  }
  
  if(timeCounter1 > 0){
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);
        data_read();
}
  if(timeCounter2 > 0){
        portENTER_CRITICAL(&timerMux);
        timeCounter2--;
        portEXIT_CRITICAL(&timerMux);
        risyou();
}
  if(timeCounter3 > 0){
        portENTER_CRITICAL(&timerMux);
        timeCounter3--;
        portEXIT_CRITICAL(&timerMux);
        kaihou();
}
}


void data_read(){ //中身はロガー電装のmain_loop関数とほとんど同じにさせてもらった．センサ対地角だけなくしてます．
  int error;
  accel_t_gyro_union accel_t_gyro;
  int32_t  temp_cal;
  uint32_t humi_cal, pres_cal;
  float temp, humi, pres, altitude;
  SD.begin(5);
  myFile = SD.open(f_name,FILE_APPEND);
  //BME280測定条件設定(1回測定後、スリープモード)
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(CTRL_MEAS);//測定条件設定
  Wire.write(0x25);//「温度・気圧オーバーサンプリングx1」、「1回測定後、スリープモード」
  Wire.endTransmission();


  //測定データ取得
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(0xF7);//出力データバイトを「気圧データ」のアドレスに指定
  Wire.endTransmission();
  
  Wire.requestFrom(BME280_ADDR, 8);//I2Cデバイス「BME280」に8Byteのデータ要求
  for (i=0; i<8; i++){
    while (Wire.available() == 0 ){}
    dac[i] = Wire.read();//dacにI2Cデバイス「BME280」のデータ読み込み
  }
  
  
  adc_P = ((uint32_t)dac[0] << 12) | ((uint32_t)dac[1] << 4) | ((dac[2] >> 4) & 0x0F);
  adc_T = ((uint32_t)dac[3] << 12) | ((uint32_t)dac[4] << 4) | ((dac[5] >> 4) & 0x0F);
  adc_H = ((uint32_t)dac[6] << 8) | ((uint32_t)dac[7]);
  
  pres_cal = BME280_compensate_P_int32(adc_P);//気圧データ補正計算
  temp_cal = BME280_compensate_T_int32(adc_T);//温度データ補正計算
  humi_cal = bme280_compensate_H_int32(adc_H);//湿度データ補正計算

  pres = (float)pres_cal / 100.0;//気圧データを実際の値に計算
  temp = (float)temp_cal / 100.0;//温度データを実際の値に計算
  humi = (float)humi_cal / 1024.0;//湿度データを実際の値に計算
  altitude = 44330.0 * (1.0 - pow(pres / SeaLevelPressure, (1.0/5.255))); 
   // 加速度、角速度の読み出し
  // accel_t_gyroは読み出した値を保存する構造体、その後ろの引数は取り出すバイト数
  error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *)&accel_t_gyro, sizeof(accel_t_gyro));

  // 取得できるデータはビッグエンディアンなので上位バイトと下位バイトの入れ替え（AVRはリトルエンディアン）
  uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

  // 取得した加速度値を分解能で割って加速度(G)に変換する
  const int arange = 4096;
  float ax = accel_t_gyro.value.x_accel / arange; //FS_SEL_2
  float ay = accel_t_gyro.value.y_accel / arange;
  float az = accel_t_gyro.value.z_accel / arange;
  Serial.print(ax, 2);
  Serial.print("\t");
  Serial.print(ay, 2);
  Serial.print("\t");
  Serial.print(az, 2);
  Serial.print("\t");

  // 取得した角速度値を分解能で割って角速度(degrees per sec)に変換する
  const int grange = 16.4;
  float gx = accel_t_gyro.value.x_gyro / grange;//FS_SEL_3
  float gy = accel_t_gyro.value.y_gyro / grange;
  float gz = accel_t_gyro.value.z_gyro / grange;
  Serial.print(gx, 2);
  Serial.print("\t");
  Serial.print(gy, 2);
  Serial.print("\t");
  Serial.print(gz, 2);
  Serial.print("\t");
  
  myFile.print(timeCounter4);
  myFile.print(",");
  myFile.print(pres);//「pres」をシリアルモニタに送信
  myFile.print(",");
  myFile.print(temp);//「temp」をシリアルモニタに送信
  myFile.print(",");//文字列「°C 」をシリアルモニタに送信
  myFile.print(humi);//「humi」をシリアルモニタに送信
  myFile.print(",");//文字列「%」をシリアルモニタに送信、改行
  myFile.print(altitude);
  myFile.print(",");
  
  myFile.print(acc_x, 2);
  myFile.print(",");
  myFile.print(acc_y, 2);
  myFile.print(",");
  myFile.print(acc_z, 2);
  myFile.print(",");
  
  myFile.print(acc_angle_x, 2);
  myFile.print(",");
  myFile.print(acc_angle_y, 2);
  myFile.print(",");
  myFile.print(acc_angle_z, 2);
  myFile.print(",");
  
  myFile.print(gyro_x, 2);
  myFile.print(",");
  myFile.print(gyro_y, 2);
  myFile.print(",");
  myFile.print(gyro_z, 2);
  myFile.print(",");
 
  Serial.print(timeCounter4);
  Serial.print("\t");
  Serial.print(pres);
  Serial.print("\t");
  Serial.print(temp);
  Serial.print("\t");
  Serial.print(humi);
  Serial.print("\t");
  Serial.print(altitude);
  Serial.print("\t");
  
  Serial.print(ax, 2);
  Serial.print("\t");
  Serial.print(ay, 2);
  Serial.print("\t");
  Serial.print(az, 2);
  Serial.print("\t");
  
  Serial.print(gx, 2);
  Serial.print("\t");
  Serial.print(gy, 2);
  Serial.print("\t");
  Serial.print(gz, 2);
  Serial.print("\t");
  //シリアルモニタ送信
 
  while (Serial2.available() > 0){
  char c = Serial2.read();
  gps.encode(c);
  if (gps.location.isUpdated()){ 
    myFile.print(gps.location.lat(), 6);
    myFile.print(","); 
    myFile.print(gps.location.lng(), 6);
    myFile.print(","); 
    myFile.print(gps.altitude.meters());
    Serial.print(gps.location.lat(), 6);
    Serial.print("\t"); 
    Serial.print(gps.location.lng(), 6);
    Serial.print("\t"); 
    Serial.print(gps.altitude.meters());
  }
 }
 myFile.println("");
 myFile.close();
 Serial.println("");  
}


void risyou(){
  int error;
  error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *)&accel_t_gyro, sizeof(accel_t_gyro));
  uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);

  const int arange = 4096;
  float ax = accel_t_gyro.value.x_accel / arange; //FS_SEL_2
  float ay = accel_t_gyro.value.y_accel / arange;
  float az = accel_t_gyro.value.z_accel / arange;
  
  //移動平均をとる
  asqrt[0] = sqrt(pow(ax, 2)+pow(ay, 2)+pow(az, 2));

  float aave = (asqrt[0]+asqrt[1]+asqrt[2]+asqrt[3]+asqrt[4]) / 5;

  for(ai = 0; ai < 5; ai++){
    _asqrt[ai] = asqrt[ai] ;
}
  for(ai = 1; ai < 5; ai++){
    asqrt[ai] = _asqrt[ai-1] ;
}
  Serial.print(aave, 2);
  Serial.println("");

  //連続回数を調べる
  if(aave > A_FLIGHT){
    acnt++;
  } else{
    acnt = 0;
  }          
  if(acnt > 4){
    Serial.print("Phaze:BURNING"); //フェーズ：BURNING
    Serial.println("");
    if (timer2){
      timerEnd(timer2);
      timer2 = NULL;
    }
  }
}


void kaihou(){ 
}

// MPU6050_read
int MPU6050_read(int start, uint8_t *buffer, int size) {
  int i, n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1) {
    return (-10);
  }
  n = Wire.endTransmission(false);// hold the I2C-bus
  if (n != 0) {
    return (n);
  }
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while (Wire.available() && i < size) {
    buffer[i++] = Wire.read();
  }
  if ( i != size) {
    return (-11);
  }
  return (0); // return : no error
}

// MPU6050_write
int MPU6050_write(int start, const uint8_t *pData, int size) {
  int n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);// write the start address
  if (n != 1) {
    return (-20);
  }
  n = Wire.write(pData, size);// write data bytes
  if (n != size) {
    return (-21);
  }
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0) {
    return (error);
  }

  return (0);// return : no error
}

// MPU6050_write_reg
int MPU6050_write_reg(int reg, uint8_t data) {
  int error;
  error = MPU6050_write(reg, &data, 1);
  Serial.print("error = ");
  Serial.println(error);
  return (error);
};

//温度補正 関数
int32_t BME280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1  = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T  = (t_fine * 5 + 128) >> 8;
  return T;
}

//湿度補正 関数
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800)); 
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
((int32_t)dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r>>12);
}

//気圧補正 関数
uint32_t BME280_compensate_P_int32(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
  var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
  var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((uint32_t)var1);
  }
  else
  {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}
