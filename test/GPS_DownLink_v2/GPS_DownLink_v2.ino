#include "TinyGPS++.h"

const int LORA_RX = 16;
const int LORA_TX = 17;
const int GPS_RX = 26;
const int GPS_TX = 27;

/*タイマー割込み処理関係の変数*/
volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer1();
void loraInit();
void gpsInit();

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  /*ES920とのシリアル通信の初期化*/
  Serial2.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(1000);
  loraInit();
  Serial2.print("GPS SET UP\r\n");
  delay(100);

  /*GNSSモジュールとのシリアル通信の初期化*/
  Serial1.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX); //事前にボーレートを115200bpsに変更しておく
  gpsInit();
  
  Serial2.print("SET UP DONE...READY\r\n");

  delay(100);

  /*タイマー割込み開始*/
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / 5.0, true);
  timerAlarmEnable(timer1); 

}

void loop() {
  interrupts();
  if(timeCounter1 > 0){
    portENTER_CRITICAL(&timerMux);
    timeCounter1--;
    portEXIT_CRITICAL(&timerMux);
    /*割込み処理*/
    while (Serial1.available() > 0){
      char c = Serial1.read();
      gps.encode(c);
      if (gps.location.isUpdated()){
        Serial2.print(gps.time.value());  Serial2.print(",");
        Serial2.print(gps.location.lat(), 6);  Serial2.print(",");
        Serial2.print(gps.location.lng(), 6); Serial2.print(",");
        Serial2.print(gps.altitude.meters());  Serial2.print("\r\n");
      }
    }
  noInterrupts();
  }
}

void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void loraInit(){
  Serial2.print("2\r\n"); //Proccesingモード
  Serial2.print("node 2\r\n");  //子機に設定
  Serial2.print("ownid 0001\r\n");  //自ノードIDを0001に設定
  Serial2.print("dstid 0000\r\n");  //送信先のIDを0000(親機)に設定
  Serial2.print("q 1\r\n"); //次回の起動時にconfigモードにする設定
  Serial2.print("w\r\n"); //設定内容の保存
  Serial2.print("z\r\n"); //通信の開始
}

void gpsInit(){
  Serial1.print("$PMTK220,100*2F\r\n");  //サンプリングレートを10[Hz]に変更
  delay(100);
  Serial1.print("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");  //GPRMCとGPGGAセンテンスのみ受信
  delay(100);
}
