#include "bme280.h"
#include <SD.h>

#define ALT_LIMIT 10  //[m]
#define VELOCITY_LIMIT 15 //[m/s]

bme280 bme(21, 22);

/*変数*/
double P0;
double alt = 0.0;
double v;
double t = 0.0;
//double t0, t1;
int reset_cnt = 0;
int judge_cnt = 0;
const double a = 0.237;
const int SR1 = 200;
const int SR2 = 100;
const char* f_name = "/test-alt.csv";
File sd;

/*タイマー割込み処理関係の変数*/
volatile int timeCounter1;
volatile int timeCounter2;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
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
  sd = SD.open("/test-alt.csv", FILE_APPEND);
  if(sd){
    sd.println("TIME, ALT, V_Z");
  }else{
    Serial.println("Opening file failed!");
    exit(1);
  }
  sd.close();
  Serial.println("initialization done.");
  
  /*BME280初期化*/
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
  
  /*タイマー割込み処理初期化*/
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / SR1, true);
  timerAlarmEnable(timer1);
  sd = SD.open(f_name, FILE_APPEND);
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 1.0E6 / SR2, true);
  timerAlarmEnable(timer2);
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
    v = (alt2 - alt) / 0.1;
    alt = alt2;
  }
  if(timeCounter2 > 0){
    portENTER_CRITICAL(&timerMux2);
    timeCounter2--;
    portEXIT_CRITICAL(&timerMux2);
    /*以下割込み処理(記録)*/
    //t0 = millis();
    double data[3];
    bme.getValue(data);
    t += 1 / (double)SR2;
    sd = SD.open(f_name, FILE_APPEND);
    if(sd){
      sd.print(t);  sd.print(",");  sd.print(data[0]);  sd.print(",");  sd.println(data[1]);
    }
    reset_cnt++;
    if(reset_cnt >= 100){
      /*1秒に一回書き込みをclose*/
      sd.close();
      sd = SD.open(f_name, FILE_APPEND);
      reset_cnt = 0;  
    }
    //t1 = millis();
    
  }
  noInterrupts();
  if((alt > ALT_LIMIT) && (v < VELOCITY_LIMIT)){
      judge_cnt++;
  }else{
    judge_cnt = 0;
  }
  if(judge_cnt > 50){
    Serial.println("Sky swimming!");
    timerEnd(timer1);
  }
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
