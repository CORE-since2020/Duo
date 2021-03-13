#include <SPI.h>
#include <SD.h>

// define
int sensorPin = A0;
int sensorValue = 0;
int n = 1;
int count;
int SAMPLING_RATE = 200;
const char* file_name = "/test.csv";
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

			timer1 = timerBegin(0, 80, true);
      timerAttachInterrupt(timer1, &onTimer1, true);
      timerAlarmWrite(timer1, 1.0E6 / SAMPLING_RATE, true);
      timerAlarmEnable(timer1);

  	Serial.begin(115200);

  	while (!SD.begin(5)) {
        	if(SD.begin(5)){
                myFile = SD.open(file_name, FILE_APPEND);
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
        if (timeCounter1 > 0) {
							portENTER_CRITICAL(&timerMux);
							timeCounter1--;
							portEXIT_CRITICAL(&timerMux);
	              	myFile = SD.open(file_name, FILE_APPEND);
	              	sensorValue = analogRead(sensorPin);
		              myFile.print(n);
		              myFile.print(",");
		              myFile.println(sensorValue);
		              myFile.close();
		              Serial.print("Done");
		              Serial.println(n);
		              ++n;
}
}
