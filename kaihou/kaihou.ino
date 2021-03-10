int i=0;
void setup()
{
  double p[]; //気圧センサから取得した気圧値を格納する
  double t[]; //温度値を格納する
  double p_ave1, p_ave2;
  double t_ave1, t_ave2;
  double h_old, h_new;
}

void loop()
{
    while(i < 10)
    {  
          p_ave1 = (p[i]+p[i+1]+p[i+2]+p[i+3]+p[i+4]) / 5;
          p_ave2 = (p[i+1]+p[i+2]+p[i+3]+p[i+4]+p[i+5])/ 5;
    
          t_ave1 = (t[i]+t[i+1]+t[i+2]+t[i+3]+t[i+4]) / 5;
          t_ave2 = (t[i+1]+t[i+2]+t[i+3]+t[i+4]+t[i+5])/ 5;
   
          h_old = (pow(1013.25/p_ave1, 1/5.257) - 1)*(t_ave1+273.15) / 0.0065;
          h_new = (pow(1013.25/p_ave2, 1/5.257) - 1)*(t_ave2+273.15) / 0.0065;

          if(h_old < h_new){
             Serial.println(h_new higher);
          }
          else{
             Serial.println(h_new lower);
             i++
          }
     }
 }
