#include "TinyGPS++.h"

const int LORA_RX = 16;
const int LORA_TX = 17;

void loraInit();
void clearBuffer();
void gpsInit();

TinyGPSPlus gps;

void setup() {
  /*ES920とのシリアル通信の初期化*/
  Serial2.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(1000);
  loraInit();
  Serial2.print("GPS SET UP\r\n");
  delay(100);

  /*GNSSモジュールとのシリアル通信の初期化*/
  Serial.begin(115200); //事前にボーレートを115200bpsに変更しておく
  gpsInit();
  
  Serial2.print("SET UP DONE...READY\r\n");

  delay(100);

}

void loop() {
  while (Serial.available() > 0){
    char c = Serial.read();
    gps.encode(c);
    if (gps.location.isUpdated()){
      Serial2.print("LAT="); Serial2.print(gps.location.lat(), 6);  Serial2.print("\r\n");
      Serial2.print("LONG="); Serial2.print(gps.location.lng(), 6); Serial2.print("\r\n");
      Serial2.print("ALT="); Serial2.print(gps.altitude.meters());  Serial2.print("\r\n");
    }
  }
}

void loraInit(){
  Serial2.print("2\r\n"); //Proccesingモード
  clearBuffer();
  Serial2.print("node 2\r\n");  //子機に設定
  clearBuffer();
  Serial2.print("ownid 0001\r\n");  //自ノードIDを0001に設定
  clearBuffer();
  Serial2.print("dstid 0000\r\n");  //送信先のIDを0000(親機)に設定
  clearBuffer();
  Serial2.print("q 1\r\n"); //次回の起動時にconfigモードにする設定
  clearBuffer();
  Serial2.print("w\r\n"); //設定内容の保存
  clearBuffer();
  Serial2.print("z\r\n"); //通信の開始
  clearBuffer();
}

void clearBuffer(){
  
}

void gpsInit(){
  Serial.print("$PMTK220,100*2F\r\n");  //サンプリングレートを10[Hz]に変更
  delay(100);
  Serial.print("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");  //GPRMCとGPGGAセンテンスのみ受信
  delay(100);
}
