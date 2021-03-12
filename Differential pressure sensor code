#include <SPI.h>
#include <SD.h>

int sensorPin = A0;
int sensorValue = 0;
int n = 1;
File myFile;

void setup() {
  Serial.begin(115200);
  while (!SD.begin(5)) {
        if(SD.begin(5)){
                myFile = SD.open("/test.txt", FILE_APPEND);
                myFile.println("START RECORD");
                myFile.println("No,SensorValue");
                myFile.close();
                Serial.println("setup Done");
         }else{
                Serial.println("ERROR");
               }
              }
}

void loop() {
  myFile = SD.open("/test.txt", FILE_APPEND);
  sensorValue = analogRead(sensorPin);
  myFile.print( n );
  myFile.print(",");
  myFile.println(sensorValue);
  myFile.close();
  Serial.print("Done");
  Serial.println(n);
  ++n;
}
