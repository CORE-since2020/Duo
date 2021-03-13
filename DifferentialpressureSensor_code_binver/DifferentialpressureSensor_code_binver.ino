#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>

// Argument definition
int sensorPin = A0;
int sensorValue[3] = 0;
int n = 1;
int count;
const int SAMPLING_RATE = 20000;
const char* file_name = "/test.bin";
File myFile;


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
  // SD card reading
  SD.begin(5);

  // Timer sys
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer1);

  // Universal Serial bus starting
  Serial.begin(115200);


  myFile = SD.open(file_name, FILE_APPEND);

      // if there is an error
      while (!SD.begin(5)) {
      Serial.println("ERROR");
    }
}

void loop() {
  if (timeCounter1 > 0) {
    portENTER_CRITICAL(&timerMux);
    timeCounter1--;
    portEXIT_CRITICAL(&timerMux);


    // read SensorValue
    sensorValue[0] = analogRead(sensorPin);
    sensorValue[1] = bmp.readTemperature() * 1000.;
    sensorValue[2] = bmp.readPressure() * 1000.;


    for (int i = 0; i<3; i++) {
      byte buf[4];
      casttobyte(sensorValue[i],buf);
      myFile.write(buf,sizeof(buf));
    }



    if((n%SAMPLING_RATE) == 0){
      Serial.print("Done");
      Serial.println(n);
      myFile.close();
      myFile = SD.open(file_name, FILE_APPEND);
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
