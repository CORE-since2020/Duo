void setup() {
  Serial.begin(115200); //HardwareSerialの用意
  Serial.print("Ready\n");
  Serial1.begin(9600, SERIAL_8N1, 26, 27);
  delay(100);
  Serial1.print("$PMTK251,115200*1F\r\n");
}

void loop() {
    if(Serial1.available()){
      Serial.write(Serial1.read());
    }
}
