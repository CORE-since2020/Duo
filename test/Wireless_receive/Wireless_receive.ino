#include <stdlib.h>

#define RECV_SIZE 30
#define LORA_RX 16
#define LORA_TX 17

String dstId = "00010001";  /*送信相手の番号*/
const int setCmdDelay = 100; /*待機時間*/

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(3500);
  loraInit();
  Serial.println("Start Recv");
}


void loop() {
  char RecvData[RECV_SIZE] = "";
  unsigned char n = 0;

  while (Serial2.available() > 0) {
    // バッファから一文字取り出す
    RecvData[n] = Serial2.read();
    // 改行文字が来たらNULL文字にする
    if (RecvData[n] == '\r' || RecvData[n] == '\n') {
      RecvData[n] = '\0';
      clearBuffer();
      break;
    }
    
    if (n < RECV_SIZE) {
      n++;
    } else {
      n = 0;
    }
  }
  //delay(300);
  if(strcmp(RecvData, "flight") == 0){
    Serial2.print("FLIGHT MODE\r\n");
  }else if(strcmp(RecvData, "info") == 0){
    Serial2.print("INFO\r\n");
  }
}


void loraInit() {
  Serial.print("Start...");
  // コマンドモード開始
  Serial2.print("2\r\n"); clearBuffer();
  // node
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
  Serial.println("Set up OK!");
}

void clearBuffer() {
  delay(setCmdDelay);
  while (Serial2.available() > 0) Serial2.read();
}
