#include <ESP_servo.h>


ESP_servo servo1;
ESP_servo servo2;


void setup(){
  servo1.init(0,0);
  servo1.write(100);

  delay(2000);

  servo2.init(2,1);
  servo2.write(150);

  delay(5000);
}


void loop(){
  for (int pos = 100; pos >= 70; pos -= 1) {
    servo1.write(pos);
    delay(5);
  }
  delay(5000);
  for (int pos = 150; pos >= 90; pos -= 1) {
    servo2.write(pos);
    delay(5);
  }
  while(1);
}
