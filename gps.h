#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>

#define GPSBAUD 9600
#define PPS null

extern TinyGPSPlus gps;

static void smartDelay(unsigned long ms);
static void printFloat(float val, bool valid, int len, int prec);
static void printInt(unsigned long val, bool valid, int len);
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);

#endif