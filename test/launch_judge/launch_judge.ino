#include "mpu6050.h"

/*タイマー割込み処理関係の変数*/
volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/*関数のプロトタイプ宣言*/
void IRAM_ATTR onTimer1();
double average(double*, int);

/*オブジェクト宣言*/
mpu6050 mpu(21, 22, 2, 3);

/*加速度閾値[G]*/
const int judge_acc = 2.5;

/*変数たち*/
double a[5];
int i = 0;
int judge_counter = 0;


void setup() {
  Serial.begin(115200);  
  mpu.setup();

  /*タイマー割込み処理開始*/ 
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
    int16_t data[3];
    mpu.getValueRawACC(data);
    a[i] = mpu.SynthesizeACC(data[0], data[1], data[2]);
    i++;
    if(i > 4){
      i = 0;
    }
  }
  noInterrupts();
    
  if(average(a, 5) >= judge_acc){
    judge_counter++;
  }else{
    judge_counter = 0;
  }
  if(judge_counter > 5){
    Serial.println("LAUNCH!");
    timerEnd(timer1);
    while(1);
  }
}

void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
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
