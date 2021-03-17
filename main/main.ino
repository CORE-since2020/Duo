/*インクルード*/
#include "mpu6050.h"
#include "bme280.h"
#include <ESP_servo.h>
#include <SD.h>

/*サンプリングレート*/
const double sampling_rate = 200.0;

/*離床判定の閾値[G]*/
const int judge_acc = 3.0;
/*離床判定のカウント数*/
const int LAUNCH_COUNT = 5;
/*燃焼時間*/
const int b_time = 4100;
/*頂点判定カウント数*/
const int OPEN_COUNT = 200;
/*頂点到達最大経過時間[ms]*/
const int timeout = 11940;
/*
memo
仰角ごとの頂点到達時間[s]
70 11.42
75 11.94
80 12.32
85 12.50
*/
/*パラ開放インターバル[ms]*/
const int open_interval = 2000;

/*ピンアサイン*/
const int cs = 5;
const int wireless_rx = 16;
const int wireless_tx = 17;
const int sda = 21;
const int scl = 22;
const int servo1= 0;
const int servo2 = 2;

/*その他の定数*/
const int acc_range_mode = 3;
const int gyro_range_mode = 2;

/*タイマー割込み処理関係の変数*/
volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/*関数のプロトタイプ宣言*/
void standby();
void launch();
void burning();
void rising();
void parachute(); 
void IRAM_ATTR onTimer1();
void LoraInit();
void clearBuffer();
double calc_altitude(double, double);
double average(double*, int);

/*フェーズの定義*/
typedef enum phase{
  STANDBY,
  LAUNCH,
  BURNING,
  RISIN,
  PARACHUTE
} phase;

phase p = STANDBY;  //フェーズの初期化

/*初期化処理かループ処理か分ける変数の定義*/
typedef enum ProsessingMode{
  INIT,
  LOOP
} ProcessingMode;

ProcessingMode pm = INIT;  //処理の種類識別の変数宣言

/*その他の変数*/
int servo1_init_pos = 100;
int servo1_fin_pos = 50;
int servo2_init_pos = 90;
int servo2_fin_pos = 0;
double s_press = 1013.20; //オフセット用の気圧
double acc[5]; //離床判定用の加速度格納配列
int j = 0;  //配列ナンバリング(加速度)
int L_judgeCounter = 0; //離床判定のカウンター
unsigned long t_zero; //離床判定確定時の時間
double alt[5];  //開放判定用の高度格納配列
int k = 0;  //配列ナンバリング(高度)
int O_judgeCounter = 0; //開放判定のカウンター
int LoadCounter = 0;  //高度の取得データが安定するまで待つためのカウンタ
double max_alt = -1000.0; //高度最大値
double new_alt; //高度更新用の変数

/*オブジェクト宣言*/
mpu6050 mpu(sda, scl, acc_range_mode, gyro_range_mode);
bme280 bme(sda, scl);
ESP_servo Servo1;
ESP_servo Servo2;

/*SD*/
File sd;

void setup() {
  /*MPU6050の初期化*/
  mpu.setup();
  
  /*BME280の初期化*/
  bme.setup();
   
  /*ES920LRの初期化*/
  Serial2.begin(115200, SERIAL_8N1, wireless_rx, wireless_tx); //無線機-ESP32
  LoraInit();
  Serial2.print("Hello! Hibari1\r\n");

  /*SDの初期化*/
  if (!SD.begin(cs)) {
    Serial2.print("SD initialization failed!\r\n");
    return;
  }
  Serial2.print("SD initialization done\r\n");
  sd = SD.open("/hibari1_log.txt", FILE_APPEND);
  if(sd){
    sd.println("Hello! Hibari1");
  }

  /*サーボモータの初期化*/
  Servo1.init(servo1, 0);
  Servo1.write(servo1_init_pos); //初期位相[deg]

  delay(2000);
  
  Servo2.init(servo2, 1);
  Servo2.write(servo2_init_pos); //初期位相[deg]

}

void loop() {
  switch(p){
    case STANDBY:
      /*スタンバイフェーズの処理*/
      standby();
      break;
      
    case LAUNCH:
      /*ローンチフェーズの処理*/
      launch();
      break;

    case BURNING:
      /*バーニングフェーズの処理*/
      burning();
      break;

    case RISIN:
      /*ライジングフェーズの処理*/
      rising();
      break;
      
    default:
      /*スタンバイフェーズに戻る*/
      p = STANDBY;
      break;
  }
}

/*スタンバイフェーズ処理関数*/
void standby(){
  char buf[30] = "";
  unsigned char i = 0;
  while(Serial2.available() > 0){
    buf[i] = Serial2.read();

    if(buf[i] == '\r' || buf[i] == '\n'){
      buf[i] = '\0';
      clearBuffer();
      break;
    }
    if(i < 30){
      i++;
    }else{
      i = 0;
    }
  }
  if(strcmp(buf, "flight") == 0){
    p = LAUNCH;
    if(sd){
      sd.println("Received Command!");
    }
    Serial2.print("FLIGHT_MODE!\r\n");
  }else if(strcmp(buf, "info") == 0){
    int16_t a[3];
    double b[3];
    mpu.getValueRawACC(a);
    double syn_acc = mpu.SynthesizeACC(a[0],a[1],a[2]);

    bme.getValue(b);
    double alt = calc_altitude(b[0],b[1],s_press);

    Serial2.print("STANDBY "); Serial2.print("ACC:");  Serial2.print(syn_acc); Serial2.print("[G] ");
    Serial2.print("ALT:");  Serial2.print(alt); Serial2.print("[m]\r\n");
  }
}

void launch(){
  switch(pm){
    case INIT:{
      /*タイマー割込み処理開始*/ 
      timer1 = timerBegin(0, 80, true);
      timerAttachInterrupt(timer1, &onTimer1, true);
      timerAlarmWrite(timer1, 1.0E6 / sampling_rate, true);
      timerAlarmEnable(timer1);  

      pm = LOOP;  //処理の種類をLOOP変更
    }break;
    case LOOP:{
      interrupts();
      if(timeCounter1 > 0){
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);
        /*割込み処理*/
        int16_t data[3];
        mpu.getValueRawACC(data);
        acc[j] = mpu.SynthesizeACC(data[0], data[1], data[2]);
        j++;
        if(j > 4){
          j = 0;
        }
      }
      noInterrupts();
      /*離床判定*/
      if(average(acc, 5) >= judge_acc){
        L_judgeCounter++;
      }else{
        L_judgeCounter = 0;
      }
      if(L_judgeCounter > LAUNCH_COUNT){
        /*離床を確認*/
        timerEnd(timer1);
        t_zero = millis();
        p = BURNING;
        pm = INIT;
        if(sd){
          sd.println("LAUNCH");
          sd.println("JUDGE ACCELERATION");
          sd.print(average(acc, 5));  sd.println("[G]");
        }
        Serial2.print("LAUNCH!\r\n");
      }
    }break;
    default:{
      /*INITに戻る*/
      pm = INIT;
    }break;
  }
}

/*バーニング処理関数*/
void burning(){
  /*燃焼中待機*/
  Serial2.print("BURNING TIME\r\n");
  delay(b_time);
  /*ライジングフェーズへ移行*/
  p = RISIN;
  Serial2.print("RISING\r\n");
}

/*ライジング処理関数*/
void rising(){
  switch(pm){
    case INIT:{
      /*タイマー割込み開始*/
      timer1 = timerBegin(0, 80, true);
      timerAttachInterrupt(timer1, &onTimer1, true);
      timerAlarmWrite(timer1, 1.0E6 / sampling_rate, true);
      timerAlarmEnable(timer1); 
      pm = LOOP;  //処理の種類をLOOP変更
    }break;
    case LOOP:{
      interrupts();
      if(timeCounter1 > 0){
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);
        /*割込み処理*/
        double data[3];
        bme.getValue(data);
        alt[j] = calc_altitude(data[0], data[1],s_press);
        k++;
        LoadCounter++;
        if(k > 4){
          k = 0;
        }
        if(LoadCounter >= 10){  //値が安定するまでのカウント
          /*開放判定*/
          new_alt = average(alt, 5);
          if(new_alt - max_alt < 0){
            O_judgeCounter++;
          }else{
            O_judgeCounter = 0;
            max_alt = new_alt;
          }
        }
      }
      noInterrupts();
      if(O_judgeCounter >= OPEN_COUNT){
        /*頂点到達を確認*/
        timerEnd(timer1);
        Serial2.print("Open\r\n");
        /*サーボ駆動*/
        for (int pos = servo1_init_pos; pos >= servo1_fin_pos; pos -= 1) {
          Servo1.write(pos);
          delay(5);
        }
        
        delay(open_interval);

        for (int pos = servo2_init_pos; pos >= servo2_fin_pos; pos -= 1) {
          Servo2.write(pos);
          delay(5);
        }

        if(sd){
          sd.println("Open judged by altitude");
        }
        
        p = PARACHUTE;
        pm = INIT;
      }else if((millis() - t_zero) >= timeout){ 
        /*タイムアウト*/
        timerEnd(timer1);
        Serial2.print("Open by timeout\r\n");
        /*サーボ駆動*/
        
         for (int pos = servo1_init_pos; pos >= servo1_fin_pos; pos -= 1) {
          Servo1.write(pos);
          delay(5);
        }
        
        delay(open_interval);

        for (int pos = servo2_init_pos; pos >= servo2_fin_pos; pos -= 1) {
          Servo2.write(pos);
          delay(5);
        }

        if(sd){
          sd.println("Open judged by timeout");
        }
        
        p = PARACHUTE;
        pm = INIT;
      }
    }break;
    default:{
      /*INITに戻る*/
      pm = INIT;
    }break;
  }
}

void parachute(){
  if(sd){
    sd.println("Finish!");
  }
  sd.close();
}

void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void LoraInit(){
  // コマンドモード開始
  Serial2.print("2\r\n"); clearBuffer();
  // 子機(End Device)に設定
  Serial2.print("node 2\r\n"); clearBuffer();
  // 自分のノードIDを設定
  Serial2.print("ownid 0001\r\n"); clearBuffer();
  //送信先のノードID
  Serial2.print("dstid 0000\r\n"); clearBuffer();
  // 起動時configuration
  Serial2.print("q 1\r\n");
  //保存
  Serial2.print("w\r\n"); clearBuffer();
  // 通信の開始
  Serial2.print("z\r\n"); clearBuffer();
}

void clearBuffer(){
  delay(100);
  while (Serial2.available() > 0) Serial2.read();
}

double calc_altitude(double pres, double temp, double offset_p){
  double alt;

  alt = (pow(offset_p /pres, 1 / 5.257) - 1) * (temp + 273.15) / 0.0065;

  return alt;
}

double average(double* e, int len){
  double sum = 0;
  double ave;
  for(int i = 0; i < len; i++){
    sum += e[i];
  }

  ave = sum / len;

  return ave;
}
