#ifndef ADXL345_H
#define ADXL345_H

#define ADXL345_CTRL_FORMAT    0x31
#define ADXL345_CTRL_FORMATV   0x03
#define ADXL345_CTRL_2D        0x2D
#define ADXL345_CTRL_2DV       0x08
#define ADXL345_OUT_X0         0x32
#define ADXL345_OUT_X1         0x33
#define ADXL345_OUT_Y0         0x34
#define ADXL345_OUT_Y1         0x35
#define ADXL345_OUT_Z0         0x36
#define ADXL345_OUT_Z1         0x37
#define ADXL345_ADDRESS        0x53

#include <Arduino.h>

void adxl345init();
void adxl345request(byte r1, byte r2);
int adxl345readX();
int adxl345readY();
int adxl345readZ();

#endif