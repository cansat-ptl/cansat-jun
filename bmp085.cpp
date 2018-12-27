#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "bmp085.h"

const unsigned char OSS = 0;
int ac1, ac2, ac3, b1, b2, mb;
unsigned int ac4;
long b5;

void bmp085init(){
	//Reading calibration values from BMP
	ac1 = bmp085ReadInt(BMP085_CAL_REG_AC1);
	ac2 = bmp085ReadInt(BMP085_CAL_REG_AC2);
	ac3 = bmp085ReadInt(BMP085_CAL_REG_AC3);
	ac4 = bmp085ReadInt(BMP085_CAL_REG_AC4);
	b1 = bmp085ReadInt(BMP085_CAL_REG_B1);
	b2 = bmp085ReadInt(BMP085_CAL_REG_B2);
	mb = bmp085ReadInt(BMP085_CAL_REG_MB);	
}
short bmp085GetTemperature(unsigned int ut){
	//Making temperature correction using calibration values
    long x1, x2;
	int mc = bmp085ReadInt(BMP085_CAL_REG_MC);
	int md = bmp085ReadInt(BMP085_CAL_REG_MD);
	unsigned int ac5 = bmp085ReadInt(BMP085_CAL_REG_AC5);
	unsigned int ac6 = bmp085ReadInt(BMP085_CAL_REG_AC6);
    x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
    x2 = ((long)mc << 11)/(x1 + md);
    b5 = x1 + x2;
    return ((b5 + 8)>>4);    
}
long bmp085GetPressure(unsigned long up){
    long x1, x2, x3, b3, b6, p;
    unsigned long b4, b7;
    b6 = b5 - 4000;
    // Calculate B3
    x1 = (b2 * (b6 * b6)>>12)>>11;
    x2 = (ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
    // Calculate B4
    x1 = (ac3 * b6)>>13;
    x2 = (b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
    b7 = ((unsigned long)(up - b3) * (50000>>OSS));
    if (b7 < 0x80000000)
        p = (b7<<1)/b4;
    else
        p = (b7/b4)<<1; 
    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;
    return p;
}
char bmp085Read(unsigned char address){
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();
    Wire.requestFrom(BMP085_ADDRESS, 1);
    while(!Wire.available())
        ;
    return Wire.read();
}
int bmp085ReadInt(unsigned char address){
    unsigned char msb, lsb;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();
    Wire.requestFrom(BMP085_ADDRESS, 2);
    while(Wire.available()<2)
        ;
    msb = Wire.read();
    lsb = Wire.read();
    return (int) msb<<8 | lsb;
}
unsigned int bmp085ReadUT(){
    unsigned int ut;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(BMP085_CTRL_UT);
    Wire.write(BMP085_CTRL_UTV);
    Wire.endTransmission();
    delay(5);
    ut = bmp085ReadInt(BMP085_OUT);
    return ut;
}
unsigned long bmp085ReadUP(){
    unsigned char msb, lsb, xlsb;
    unsigned long up = 0;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(BMP085_CTRL_UP);
    Wire.write(BMP085_CTRL_UPV + (OSS<<6));
    Wire.endTransmission();
    delay(2 + (3<<OSS));
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(BMP085_OUT);
    Wire.endTransmission();
    Wire.requestFrom(BMP085_ADDRESS, 3);
    while(Wire.available() < 3)
        ;
    msb = Wire.read();
    lsb = Wire.read();
    xlsb = Wire.read();
    up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS); 
    return up;
}
int calcAltitude(int prs){
	if (prs == 0) return -1;
    //weird contraptions to make everything int
	return (int) (44330.0 * (1.0 - pow((float) prs / ASL, 0.1903)))*10;
}