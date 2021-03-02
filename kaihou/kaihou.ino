
void setup()
{
  double p[]; //気圧センサから取得した気圧値を格納する
  double t[]; //温度値を格納する
  double ave_old, ave_new;
  double h_old, h_new;
  int i;
}

void loop()
{
//気圧センサから取得した気圧値の5回移動平均が１つ前の値より大きいことを10回連続で確認する．（最高点に到達し高度が下がっていることを確認する）
    ave_old = (p[i]+p[i+1]+p[i+2]+p[i+3]+p[i+4]) / 5;
    ave_new = (p[i+1]+p[i+2]+p[I+3]+p{i+4]+p[i+5])/ 5;
    if(ave_old < ave_new ){
      printf("higher\n");
    }
    h_old = (pow(1013.25/ave_old, 1/5.257) - 1)*(t[i]+273.15) / 0.0065;
    h_new = (pow(1013.25/ave_new, 1/5.257) - 1)*(t[i+1]+273.15) / 0.0065;
    if(h_old > h_new){
      printf("lower\n");
    }
 }
