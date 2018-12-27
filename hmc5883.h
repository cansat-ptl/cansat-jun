#ifndef HMC5883_H
#define HMC5883_H

#define HMC5883_ADDRESS        0x1E
#define HMC5883_CTRL_MODE      0x02
#define HMC5883_CTRL_MODEV     0x00
#define HMC5883_OUT_X          0x03
#define HMC5883_OUT_Y          0x05
#define HMC5883_OUT_Z          0x07

#include <Arduino.h>

void HMC5883init();
void HMC5883request(byte r1);
int HMC5883readX();
int HMC5883readY();
int HMC5883readZ();

#endif