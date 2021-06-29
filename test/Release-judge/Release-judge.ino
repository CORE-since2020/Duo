#include <Wire.h>
#include "VL53L0X.h"

VL53L0X VL(21, 22);
void setup()
{
  Serial.begin(115200);

  VL.setTimeout(500);
  if (!VL.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
}

void loop()
{
  //Serial.println(VL.readRangeSingleMillimeters());
  
  if(VL.readRangeSingleMillimeters() > 100){
    Serial.println("Release!");
    while(1);
  }
  
  if (VL.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
}
