#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250_asukiaaa.h>
#include <TinyGPS++.h>

// define
const int SAMPLING_RATE = 200;
const char* file_name = "/LoggerData.bin";
int64_t sensorValue[14];
File myFile;
Adafruit_BME280 bme;
MPU9250_asukiaaa mySensor;
unsigned bmpstatus;
int64_t datanumber = 0;
TinyGPSPlus gps;

#define SDA_PIN 21
#define SCL_PIN 22

// unsigned long time;
// float aX, aY, aZ, aSqrt;

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
    //*********Communications*********
    bmpstatus = bme.begin(0x76);  // SensorID:118
    SD.begin(5);
    #ifdef _ESP32_HAL_I2C_H_ // For ESP32
      Wire.begin(SDA_PIN, SCL_PIN);
      mySensor.setWire(&Wire);
    #endif
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);

  //*********SamplingData_Timer*********
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer1);
  pinMode(27,OUTPUT);


    // You can set your own offset for mag values
    // mySensor.magXOffset = -50;
    // mySensor.magYOffset = -55;
    // mySensor.magZOffset = -10;
  
  myFile = SD.open(file_name, FILE_APPEND);
}

void loop() {
    if (timeCounter1 > 0) {
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);
        
        mySensor.accelUpdate();

        char c = Serial2.read();
        gps.encode(c);
        gps.location.isUpdated();
        mySensor.gyroUpdate();
        
        sensorValue[0] = datanumber;
        sensorValue[1] = millis();
        sensorValue[2] = bme.readTemperature() * 10000;
        sensorValue[3] = bme.readPressure() * 10000;
        sensorValue[4] = bme.readHumidity() * 10000;
        sensorValue[5] = mySensor.accelX() * 10000;
        sensorValue[6] = mySensor.accelY() * 10000;
        sensorValue[7] = mySensor.accelZ() * 10000;
        sensorValue[8] = mySensor.gyroX() * 10000;
        sensorValue[9] = mySensor.gyroY() * 10000;
        sensorValue[10] = mySensor.gyroZ() * 10000;
        sensorValue[11] = gps.location.lat()*1000000000;
        sensorValue[12] = gps.location.lng()*1000000000;
        sensorValue[13] = gps.time.value();


        for (int i = 0; i<14; i++) {
            byte buf[8];
            casttobyte(sensorValue[i],buf);
            myFile.write(buf,sizeof(buf));
        }

        if (datanumber%SAMPLING_RATE==0){
          digitalWrite(27,LOW);
          myFile.close();
          myFile = SD.open(file_name, FILE_APPEND);

          for (int i = 0; i<14; i++) {
              Serial.println(sensorValue[i]);
          }
        }
        datanumber++;
    }
}

void casttobyte(int64_t data, byte buf[]){
    buf[0] = (data >> 56) & 0x00FF;
    buf[1] = (data >> 48) & 0x00FF;
    buf[2] = (data >> 40) & 0x00FF;
    buf[3] = (data >> 32) & 0x00FF;
    buf[4] = (data >> 24) & 0x00FF;
    buf[5] = (data >> 16) & 0x00FF;
    buf[6] = (data >> 8) & 0x00FF;
    buf[7] = (data) & 0x00FF;
}
