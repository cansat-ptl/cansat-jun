#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>

#define GPSBAUD 9600
#define PPS null

extern TinyGPSPlus gps;

void smartDelay(unsigned long ms);
void printFloat(float val, bool valid, int len, int prec);
void printInt(unsigned long val, bool valid, int len);
void printDateTime(TinyGPSDate &d, TinyGPSTime &t);

#endif