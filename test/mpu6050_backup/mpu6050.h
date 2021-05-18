#ifndef _MPU6050_H
#define _MPU6050_H

#include "Arduino.h"
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_CONFIG_REG 0x1A

#define MPU6050_GYRO_CONFIG_REG 0x1B
#define MPU6050_ACCELERO_CONFIG_REG 0x1C

#define MPU6050_ACCEL_XOUT_H_REG 0x3B
#define MPU6050_ACCEL_YOUT_H_REG 0x3D
#define MPU6050_ACCEL_ZOUT_H_REG 0x3F

#define MPU6050_GYRO_XOUT_H_REG 0x43
#define MPU6050_GYRO_YOUT_H_REG 0x45
#define MPU6050_GYRO_ZOUT_H_REG 0x47

class mpu6050 {
public:
	//Initialization
	mpu6050(int , int, int, int);    //(sda_pin, scl_pin, a_mode, g_mode)
    /*
    a_mode：加速度のレンジ設定
            0：±2[G]
            1：±4[G]
            2：±8[G]
            3：±16[G]
    g_mode：角速度のレンジ設定
            0：±250[deg/s]
            1：±500[deg/s]
            2：±1000[deg/s]
            3：±2000[deg/s]
    */

	//Setup
	void setup(void);

	//Get I2C Device Addess
	char getID(void);

    //Mode select
    void ModeSelect(void);  
    /*
    At a_mode = 0 Acceleration's modify = 16384
    At a_mode = 1 Acceleration's modify = 8192
    At a_mode = 2 Acceleration's modify = 4096
    At a_mode = 3 Acceleration's modify = 2048

    At g_mode = 0 Gyro's modify = 131
    At g_mode = 1 Gyro's modify = 65.5
    At g_mode = 2 Gyro's modify = 32.8
    At g_mode = 3 Gyro's modify = 16.4
    */

	//Read
	void write(byte, byte);
	//Write
	void read(byte, char*, int);

	//Get the Raw Data(Acceleration)
	void getValueRawACC(int16_t*);

    //Get the Raw Data(Gyro)
    void getValueRawGYRO(int16_t*);

    //Get the synthesize acceleration
    double SynthesizeACC(int16_t, int16_t, int16_t);    //(ACC_X, ACC_Y, ACC_Z)
	

	

private:
	  int a_mode;
    int g_mode;
    double a_modify;
    // double g_modify;
};

#endif
