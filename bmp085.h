#ifndef BMP085_H
#define BMP085_H

#define BMP085_ADDRESS         0x77
#define BMP085_CTRL_ENAB       0x00
#define BMP085_CTRL_MODE       0x02
#define BMP085_CTRL_UT         0xF4
#define BMP085_CTRL_UTV        0x2E
#define BMP085_CTRL_UP         0xF4
#define BMP085_CTRL_UPV        0x34

#define BMP085_CAL_REG_AC1     0xAA
#define BMP085_CAL_REG_AC2     0xAC
#define BMP085_CAL_REG_AC3     0xAE
#define BMP085_CAL_REG_AC4     0xB0
#define BMP085_CAL_REG_AC5     0xB2
#define BMP085_CAL_REG_AC6     0xB4
#define BMP085_CAL_REG_B1      0xB6
#define BMP085_CAL_REG_B2      0xB8
#define BMP085_CAL_REG_MB      0xBA
#define BMP085_CAL_REG_MC      0xBC
#define BMP085_CAL_REG_MD      0xBE

#define BMP085_OUT             0xF6

#define ASL 101325.0F

#include <Arduino.h>

int bmp085init();
short bmp085GetTemperature(unsigned int ut);
long bmp085GetPressure(unsigned long up);
char bmp085Read(unsigned char address);
int bmp085ReadInt(unsigned char address);
unsigned int bmp085ReadUT();
unsigned long bmp085ReadUP();
int calcAltitude(int prs);
#endif