#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// define
int sensorPin = A5;
int n = 1;
int count;
const int SAMPLING_RATE = 200;
const char* file_name = "/nose_data.bin";
int sensorValue[3];
File myFile;
Adafruit_BME280 bme;

// Interrupt timer
volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Interrupt timer function
void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() {
  bme.begin(0x76);
  SD.begin(5);
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer1);
  Serial.begin(115200);
  pinMode(27,OUTPUT);
  
  myFile = SD.open(file_name, FILE_APPEND);
  for (int i = 0; i<3; i++) {
      byte buf[4];
      casttobyte(4294967295,buf);
      myFile.write(buf,sizeof(buf));
    }
  myFile.close();
  
  while (!SD.begin(5)){
    digitalWrite(27,HIGH);
    delay(2000);
  }

  myFile = SD.open(file_name, FILE_APPEND);
}

void loop() {
  if (timeCounter1 > 0) {
    portENTER_CRITICAL(&timerMux);
    timeCounter1--;
    portEXIT_CRITICAL(&timerMux);

      sensorValue[0] = analogRead(sensorPin);
      sensorValue[1] = bme.readTemperature() * 100;
      sensorValue[2] = bme.readPressure() * 100;

    for (int i = 0; i<3; i++) {
        byte buf[4];
        casttobyte(sensorValue[i],buf);
        myFile.write(buf,sizeof(buf));
      }

    if (n%SAMPLING_RATE == 0){
      digitalWrite(27,LOW);
      Serial.print("Done");
      Serial.print(n);
      Serial.print("\t");
      Serial.print("Diffe_Press: ");
      Serial.print(sensorValue[0]);
      Serial.print("\t");
      Serial.print("Temperature: ");
      Serial.print(sensorValue[1]);
      Serial.print("\t");
      Serial.print("Pressure: ");
      Serial.print(sensorValue[2]);
      Serial.print("\n");
      myFile.close();
      myFile = SD.open(file_name, FILE_APPEND);
      if (myFile == false) {
        digitalWrite(27,HIGH);
        SD.end();
        SD.begin(5);
        myFile = SD.open(file_name, FILE_APPEND);
        
        for (int i = 0; i<3; i++) {
          byte buf[4];
          casttobyte(4294967295,buf);
          myFile.write(buf,sizeof(buf));
    }
        
        }
    }
      ++n;
}
}


void casttobyte(int32_t data, byte buf[]){
  buf[0] = (data >> 24) & 0x00FF;
  buf[1] = (data >> 16) & 0x00FF;
  buf[2] = (data >> 8) & 0x00FF;
  buf[3] = (data) & 0x00FF;
}
