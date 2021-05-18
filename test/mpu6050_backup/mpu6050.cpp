#include "mpu6050.h"

mpu6050::mpu6050(int sda, int scl, int mode_1, int mode_2){
    a_mode = mode_1;    //Acceleration's range selection
    g_mode = mode_2;    //Gyro's range selection

    Wire.begin(sda, scl);
}

//Setup
void mpu6050::setup(void) {
    write(MPU6050_PWR_MGMT_1_REG, 0x00);    //Release Sleep Mode 
    ModeSelect();   //Mode Select
    write(MPU6050_CONFIG_REG, 0x00);    //Config
}

//Get I2C Device Address
char mpu6050::getID(void) {
    char devid[1];
    read(MPU6050_WHO_AM_I_REG, devid, 1);

    return devid[0];
}

//Mode select
void mpu6050::ModeSelect(){
    switch (a_mode) {
        case 0:
            write(MPU6050_ACCELERO_CONFIG_REG, 0x00);   //Full Scale Range +-2[G]
            a_modify = 16384.0;
            break;

        case 1:
            write(MPU6050_ACCELERO_CONFIG_REG, 0x08);   //Full Scale Range +-4[G]
            a_modify = 8192.0;
            break;

        case 2:
            write(MPU6050_ACCELERO_CONFIG_REG, 0x10);   //Full Scale Range +-8[G]
            a_modify = 4096.0;
            break;

        case 3:
            write(MPU6050_ACCELERO_CONFIG_REG, 0x18);   //Full Scale Range +-16[G]
            a_modify = 2048.0;
            break;
    }

    switch (g_mode) {
        case 0:
            write(MPU6050_GYRO_CONFIG_REG, 0x00);   //Full Scale Range +-250[deg/s]
            //g_modify = 131.0;
            break;

        case 1:
            write(MPU6050_GYRO_CONFIG_REG, 0x08);   //Full Scale Range +-500[deg/s]
            //g_modify = 65.5;
            break;

        case 2:
            write(MPU6050_GYRO_CONFIG_REG, 0x10);   //Full Scale Range +-1000[deg/s]
            //g_modify = 32.8;
            break;

        case 3:
            write(MPU6050_GYRO_CONFIG_REG, 0x18);   //Full Scale Range +-2000[deg/s]
            //g_modify = 16.4;
            break;
    }
}

//Write
void mpu6050::write(byte address, byte data) {

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

//Read
void mpu6050::read(byte address, char* value, int len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(address);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, len, true);
    while (Wire.available() < len);
    for(int i = 0; i < len; i++){
        value[i] = Wire.read();
    }
}

//Get the Raw Data (Acceleration)
void mpu6050::getValueRawACC(int16_t* acc) {
    char x_data[2];
    read(MPU6050_ACCEL_XOUT_H_REG, x_data, 2);
    acc[0] = (x_data[0] << 8) | x_data[1];
    
    char y_data[2];
    read(MPU6050_ACCEL_YOUT_H_REG, y_data, 2);
    acc[1] = (y_data[0] << 8) | y_data[1];

    char z_data[2];
    read(MPU6050_ACCEL_ZOUT_H_REG, z_data, 2);
    acc[2] = (z_data[0] << 8) | z_data[1];

}

//Get the Raw Data (Gyro)
void mpu6050::getValueRawGYRO(int16_t* gyro) {
    char x_data[2];
    read(MPU6050_GYRO_XOUT_H_REG, x_data, 2);
    gyro[0] = (x_data[0] << 8) | x_data[1];
    
    char y_data[2];
    read(MPU6050_GYRO_YOUT_H_REG, y_data, 2);
    gyro[1] = (y_data[0] << 8) | y_data[1];

    char z_data[2];
    read(MPU6050_GYRO_ZOUT_H_REG, z_data, 2);
    gyro[2] = (z_data[0] << 8) | z_data[1];

}

//Calcuration of Synthesize Acceleration
double mpu6050::SynthesizeACC(int16_t acc_rawx, int16_t acc_rawy, int16_t acc_rawz){
    double acc_x, acc_y, acc_z, syn;

    acc_x = acc_rawx / a_modify;
    acc_y = acc_rawy / a_modify;
    acc_z = acc_rawz / a_modify;

    syn = sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));

    return syn;
}
