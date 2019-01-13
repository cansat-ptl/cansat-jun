#include "gps.h"
#include <Arduino.h>

TinyGPSPlus gps;

static void smartDelay(unsigned long ms){
    unsigned long start = millis();
    do {
      while (Serial.available())
          gps.encode(Serial.read());
    }   while (millis() - start < ms);
}
static void printFloat(float val, bool valid, int len, int prec){
    if (!valid){
        while (len-- > 1)
            Serial1.print('*');
        Serial1.print(' ');
    }
    else {
        Serial1.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
        for (int i=flen; i<len; ++i)
            Serial1.print(' ');
    }
    smartDelay(0);
}
static void printInt(unsigned long val, bool valid, int len){
    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i=strlen(sz); i<len; ++i)
        sz[i] = ' ';
    if (len > 0) 
        sz[len-1] = ' ';
    Serial1.print(sz);
    smartDelay(0);
}
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t){
    if (!d.isValid()){
        Serial1.print(F("NO_DATA "));
    }
    else {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        Serial1.print(sz);
    }
    if (!t.isValid()){
        Serial1.print(F("NO_DATA "));
    }
    else {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        Serial1.print(sz);
    }
    printInt(d.age(), d.isValid(), 5);
    smartDelay(0);
}