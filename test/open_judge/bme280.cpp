#include "bme280.h"

bme280::bme280(int sda, int scl){
    Wire.begin(sda, scl);
}

void bme280::setup(void) {
    write(BME280_CTRL_HUM, 0x00);
    write(BME280_CTRL_MEAS, 0x2F);
    write(BME280_CONFIG, 0xD0);

    set_calib();
}

//Get ID registor
char bme280::getID(void) {
    char devid[1];
    read(BME280_WHO_I_AM, devid, 1);

    return devid[0];
}

//Write
void bme280::write(byte address, byte data) {
    Wire.beginTransmission(BME280_ADDR);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

//Read
void bme280::read(byte address, char* value, int len) {
    Wire.beginTransmission(BME280_ADDR);
    Wire.write(address);
    Wire.endTransmission(false);
    Wire.requestFrom(BME280_ADDR, len, true);
    while (Wire.available() < len);
    for(int i = 0; i < len; i++){
        value[i] = Wire.read();
    }
}

//Get the calibrated data
void bme280::set_calib(void) {
    char buf[26];

    read(BME280_CALIB_1, buf, 26);
    dig_T1 = ((uint16_t)((buf[1] << 8) | buf[0]));
    dig_T2 = ((int16_t)((buf[3] << 8) | buf[2]));
    dig_T3 = ((int16_t)((buf[5] << 8) | buf[4]));

    dig_P1 = ((uint16_t)((buf[7] << 8) | buf[6]));
    dig_P2 = ((int16_t)((buf[9] << 8) | buf[8]));
    dig_P3 = ((int16_t)((buf[11] << 8) | buf[10]));
    dig_P4 = ((int16_t)((buf[13] << 8) | buf[12]));
    dig_P5 = ((int16_t)((buf[15] << 8) | buf[14]));
    dig_P6 = ((int16_t)((buf[17] << 8) | buf[16]));
    dig_P7 = ((int16_t)((buf[19] << 8) | buf[18]));
    dig_P8 = ((int16_t)((buf[21] << 8) | buf[20]));
    dig_P9 = ((int16_t)((buf[23] << 8) | buf[22]));

    dig_H1 = ((uint8_t)(buf[25]));

    read(BME280_CALIB_2, buf, 7);
    dig_H2 = ((int16_t)((buf[1] << 8) | buf[0]));
    dig_H3 = ((uint8_t)(buf[2]));
    dig_H4 = ((int16_t)((buf[3] << 4)+(buf[4] & 0x0F)));
    dig_H5 = ((int16_t)((buf[5] << 4) + ((buf[4] >> 4) & 0x0F)));
    dig_H6 = ((int8_t)buf[6]);
}

//Calibration
int32_t bme280::calc_temperature(int32_t adc_T) {
    int32_t v1, v2, T;
    v1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1)))* ((int32_t)dig_T2)) >> 11;
    v2 = (((((adc_T >> 4) - ((int32_t)dig_T1))* ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12)* ((int32_t)dig_T3)) >> 14;
    t_fine = v1 + v2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}
uint32_t bme280::calc_pressure(int32_t adc_P) {
    int32_t v1, v2;
    uint32_t P;
    v1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    v2 = (((v1 >> 2)* (v1 >> 2)) >> 11)* ((int32_t)dig_P6);
    v2 = v2 + ((v1 * ((int32_t)dig_P5)) << 1);
    v2 = (v2 >> 2) + (((int32_t)dig_P4) << 16);
    v1 = (((dig_P3 * (((v1 >> 2)* (v1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * v1) >> 1)) >> 18;
    v1 = (((32768 + v1) * ((int32_t)dig_P1)) >> 15);
    if (v1 == 0) {
        return 0;
    }
    P = ((uint32_t)(((int32_t)1048576) - adc_P) - (v2 >> 12)) * 3125;
    if (P < 0x80000000) {
        P = (P << 1) / ((uint32_t)v1);
    }
    else {
        P = (P / (uint32_t)v1) * 2;
    }
    v1 = (((int32_t)dig_P9) * ((int32_t)(((P >> 3)* (P >> 3)) >> 13))) >> 12;
    v2 = (((int32_t)(P >> 2))* ((int32_t)dig_P8)) >> 13;
    P = (uint32_t)((int32_t)P + ((v1 + v2 + dig_P7) >> 4));
    return P;
}

uint32_t bme280::calc_humidity(int32_t adc_H){
    int32_t v;
    v = (t_fine - ((int32_t)76800));
    v = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v)) + ((int32_t)16384.0)) >> 15) * (((((((v * ((int32_t)dig_H6)) >> 10) * (((v * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v = (v < 0 ? 0 :v);
    v = (v > 419430400 ? 419430400 : v);
    return (uint32_t)(v >> 12);
}

void bme280::getValue(double* value) {
    char data_pres[3], data_temp[3], data_humid[2];

    read(BME280_PRES_H_REG, data_pres, 3);
    read(BME280_TEMP_H_REG, data_temp, 3);
    read(BME280_HUMID_H_REG, data_humid, 2);

    adc_P = ((uint32_t)data_pres[0] << 12) | ((uint32_t)data_pres[1] << 4) | ((data_pres[2] >> 4) & 0x0F);
    adc_T = ((uint32_t)data_temp[0] << 12) | ((uint32_t)data_temp[1] << 4) | ((data_temp[2] >> 4) & 0x0F);
    adc_H = ((uint32_t)data_humid[0] << 8) | ((uint32_t)data_humid[1]);

    value[0] = (double)calc_pressure(adc_P) / 100.0;
    value[1] = (double)calc_temperature(adc_T) / 100.0;
    value[2] = (double)calc_humidity(adc_H) / 1024.0;

}
