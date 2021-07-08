#include <SD.h>
#include "bme280.h"

#define ALT_LIMIT 10  //[m]
#define VELOCITY_LIMIT 15 //[m/s]
#define INTERVAL_TIME 5000  //[ms]

bme280 bme(21, 22);

/*変数*/
double P0;
double alt = 0.0;
double v;
double t = 0.0; 
double t0;
int button_cnt = 0;
int cnt = 0;
int reset_t = 1;
int judge_cnt = 0;
const double a = 0.237;
const int SR1 = 200;
const int SR2 = 100;
const char* f_name1 = "/test-alt.csv";
File sd1;

/*タイマー割込み処理(判定)関係の変数*/
volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

/*タイマー割込み処理(記録)関係の変数*/
volatile int timeCounter2;
hw_timer_t *timer2 = NULL;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;

/*関数*/
double calc_altitude(double, double, double);
double average(double*, int);
double median(double*, int);
void IRAM_ATTR onTimer1();
void IRAM_ATTR onTimer2();

void setup() {
  /*シリアル通信初期化*/
  Serial.begin(115200);
  /*SD初期化*/
  Serial.print("Initializing SD card...");
  if (!SD.begin(5)) {
    Serial.println("initialization failed!");
    exit(1);
  }
  sd1 = SD.open(f_name1, FILE_APPEND);
  if(sd1){
    sd1.println("TIME, ALT, TEMP");
  }else{
    Serial.println("Opening file failed!");
    exit(1);
  }
  sd1.close();
  Serial.println("initialization done.");
  
  /*BME280セットアップ*/
  bme.setup();
  delay(500);
  Serial.println("Pressure Offsetting");
  double p_off[100];
  for(int i = 0; i < 100 ; i++){
    double data[3];
    bme.getValue(data);
    p_off[i] = data[0];
    delay(50);
  }
  P0 = median(p_off, 100);
  Serial.print(P0);
  Serial.println("[hPa]");
  while(button_cnt < 5){
    if(digitalRead(13) == 1){
      button_cnt++;
    }
  }
  Serial.println("Released...");

  /*タイマー割込み処理初期化(記録)*/
  sd1 = SD.open(f_name1, FILE_APPEND);
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 1.0E06 / (double)SR2, true);
  timerAlarmEnable(timer2);

  delay(INTERVAL_TIME);
  
  Serial.println("Start judgement");
  
  /*タイマー割込み処理初期化(判定)*/
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / (double)SR1, true);
  timerAlarmEnable(timer1);
  
}

void loop() {
  interrupts();
  if(timeCounter1 > 0){
    portENTER_CRITICAL(&timerMux1);
    timeCounter1--;
    portEXIT_CRITICAL(&timerMux1);
    /*以下割込み処理(判定)*/
    double data[3];
    double alt2;
    bme.getValue(data);
    alt2 = a * alt + (1 - a) * calc_altitude(P0, data[0], data[1]);  //RCフィルター
    v = (alt2 - alt) / 0.01;
    alt = alt2;
  }
  if(timeCounter2 > 0){
    portENTER_CRITICAL(&timerMux2);
    timeCounter2--;
    portEXIT_CRITICAL(&timerMux2);
    /*以下割込み処理(記録)*/
    if(cnt == 0){
      t0 =millis();
      cnt++;
    }else{
      t = (millis() - t0) * 0.001;
    }
    double data[3];
    bme.getValue(data);
    if(sd1){
      sd1.print(t);  sd1.print(",");  sd1.print(data[0]);  sd1.print(",");  sd1.println(data[1]);
    }
  }
  noInterrupts();
  if(t>=reset_t){
    sd1.close();
    sd1 = SD.open(f_name1, FILE_APPEND);
    reset_t++; 
  }

  /*
  if((alt > ALT_LIMIT) && (v < VELOCITY_LIMIT)){
      judge_cnt++;
  }else{
    judge_cnt = 0;
  }
  if(judge_cnt > 50){
    Serial.println("Sky swimming!");
    sd2 = SD.open(f_name2, FILE_APPEND);
    if(sd2){
      sd2.println("Sky swimming!");
      sd2.print("time:"); sd2.print(t); sd2.println("[s]");
    }
    sd2.close();
    timerEnd(timer1);
  }
  */
  
}

double calc_altitude(double pres_offset, double pres, double temp){
  double ans;

  ans = (pow(pres_offset /pres, 1 / 5.257) - 1) * (temp + 273.15) / 0.0065;

  return ans;
}

double median(double e[], int len){
  double *data_cpy, ans;
    data_cpy = new double[len];
    memcpy(data_cpy,e,sizeof(double)*len);
 
    for(int i = 0; i < len; i++){
        for(int j=0; j<len-i-1; j++){
            if(data_cpy[j]>data_cpy[j+1]){
                float buff = data_cpy[j+1];
                data_cpy[j+1] = data_cpy[j];
                data_cpy[j] = buff;
            }
        }
    }
    
    if(len%2!=0) ans = data_cpy[len/2];
    else         ans = (data_cpy[len/2-1]+data_cpy[len/2])/2.0;
    delete[] data_cpy;
    return ans;
}

void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timerMux1);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR onTimer2(){
  portENTER_CRITICAL_ISR(&timerMux2);
  timeCounter2++;
  portEXIT_CRITICAL_ISR(&timerMux2);
}
