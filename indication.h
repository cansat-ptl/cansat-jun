#ifndef INDICATION_H
#define INDICATION_H

#define S_DATA 9
#define S_CLK 10
#define S_SCLK 11
#define S_MR 5
#define BAT A0
#define LED1 13

#define USE_MATRIX 0

#include <Arduino.h>
#include <matrix8.h>

void handleIndicators();

#endif
