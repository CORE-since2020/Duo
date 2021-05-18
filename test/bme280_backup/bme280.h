#ifndef _BME280_H
#define _BME280_H

#include "Arduino.h"
#include <Wire.h>

#define BME280_ADDR 0x76

#define BME280_CTRL_HUM 0xF2
#define BME280_CTRL_MEAS 0xF4
#define BME280_CONFIG 0xF5

#define BME280_CALIB_1 0x88
#define BME280_WHO_I_AM 0xD0
#define BME280_CALIB_2 0xE1

#define BME280_PRES_H_REG 0xF7
#define BME280_TEMP_H_REG 0xFA
#define BME280_HUMID_H_REG 0xFD

class bme280 {
public:
	//Initialization
	bme280(int sda, int scl);

	//Setup
	void setup(void);

	//Get ID registor
	char getID(void);

	//Read
	void write(byte address, byte data);
	//Write
	void read(byte address, char* value, int len);

	//Get the calibrated data
	void set_calib(void);

	//Calibration
	int32_t calc_temperature(int32_t);
	uint32_t calc_pressure(int32_t);
	uint32_t calc_humidity(int32_t);

	//Get the Data
	void getValue(double*);

private:
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

  uint16_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;

	int32_t t_fine;
	int32_t adc_T;
	int32_t adc_P;
  int32_t adc_H;

};

#endif
