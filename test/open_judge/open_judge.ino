#include "bme280.h"

bme280 bme(21, 22);

double offset_p;
int judge_Counter = 0;
int j = 0;
int i = 0;
double alt[5];
double max_alt = -1000.0;
double new_alt;

/*タイマー割込み処理関係の変数*/
volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

double calc_altitude(double, double);
double average(double*, int);
void IRAM_ATTR onTimer1();

void setup() {
  Serial.begin(115200);

  bme.setup();
  
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / 200.0, true);
  timerAlarmEnable(timer1);  
}

void loop() {
  interrupts();
  if(timeCounter1 > 0){
    portENTER_CRITICAL(&timerMux);
    timeCounter1--;
    portEXIT_CRITICAL(&timerMux);
    /*割込み処理*/
    double data[3];
    bme.getValue(data);
    alt[j] = calc_altitude(data[0], data[1]);
    j++;
    i++;
    if(j > 4){
      j = 0;
    }
    if(i >= 10){  //値が安定するまでのカウント
      /*開放判定*/
      new_alt = average(alt, 5);
      if(new_alt - max_alt < 0){
        judge_Counter++;
      }else{
        judge_Counter = 0;
        max_alt = new_alt;
      }
    }
  }
  noInterrupts();
  if(judge_Counter >= 100){
    /*頂点到達を確認*/
    timerEnd(timer1);
    Serial.println("Open!");
    while(1);
  }
}

double calc_altitude(double pres, double temp){
  double alt;

  alt = (pow(1013.25 /pres, 1 / 5.257) - 1) * (temp + 273.15) / 0.0065;

  return alt;
}

double average(double* e, int len){
  double sum = 0;
  double ave;
  for(int i = 0; i < len; i++){
    sum += e[i];
  }

  ave = sum / len;

  return ave;
}

void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
