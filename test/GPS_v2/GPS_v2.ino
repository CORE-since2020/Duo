#include <SD.h>
#include "TinyGPS++.h"

const int LORA_RX = 16;
const int LORA_TX = 17;
const int GPS_RX = 26;
const int GPS_TX = 27;

void loraInit();
void gpsInit();

File fp;

TinyGPSPlus gps;

void setup() {
  /*SDの初期化*/
  Serial.begin(115200);
  Serial.print("Initializing SD card...");
  if (!SD.begin(5)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  fp = SD.open("/gps.csv", FILE_APPEND);

  if (fp) {
    Serial.println("Opening file!");
    fp.println("UTC, LAT, LONG, ALT");
    fp.close();
  } else {
    Serial.println("error opening test.txt");
  }

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
  
}

void loop() {
  while (Serial1.available() > 0){
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()){
      Serial2.print(gps.time.value());  Serial2.print(",");
      Serial2.print(gps.location.lat(), 6);  Serial2.print(",");
      Serial2.print(gps.location.lng(), 6); Serial2.print(",");
      Serial2.print(gps.altitude.meters());  Serial2.print("\r\n");

      /*SDへの書き込み*/
      fp = SD.open("/gps.csv", FILE_APPEND);
      fp.print(gps.time.value());  fp.print(",");
      fp.print(gps.location.lat(), 6);  fp.print(",");
      fp.print(gps.location.lng(), 6); fp.print(",");
      fp.print(gps.altitude.meters());  fp.print("\n");
    }
  }
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
