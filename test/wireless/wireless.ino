#define LORA_RX 16
#define LORA_TX 17

const int setCmdDelay = 100; /*待機時間*/


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);
  loraInit();
}

void loop() {
  Serial2.print("ONJUKU LOVE\r\n"); 
  delay(200);
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
