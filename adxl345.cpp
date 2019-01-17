#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "adxl345.h"

int adxl345init(){
	Wire.beginTransmission(ADXL345_ADDRESS);
	if(Wire.write(ADXL345_CTRL_2D) == 0) return -1;
	Wire.write(ADXL345_CTRL_2DV);     
	Wire.endTransmission();	
	delay(50);
	Wire.beginTransmission(ADXL345_ADDRESS);
	Wire.write(ADXL345_CTRL_FORMAT);
	Wire.write(ADXL345_CTRL_FORMATV);   	
	Wire.endTransmission();
	return 0;
}
void adxl345request(byte r1, byte r2){
	Wire.beginTransmission(ADXL345_ADDRESS); // transmit to device
	Wire.write(r1);
	Wire.write(r2);
	Wire.endTransmission();
	Wire.requestFrom(ADXL345_ADDRESS ,2);
}
int adxl345readX(){
	int X0 = 0,X1 = 0,X_out = 0;
	double Xg;
	adxl345request(ADXL345_OUT_X0, ADXL345_OUT_X1);
	if(Wire.available()<=2){
		X0 = Wire.read();
		X1 = Wire.read(); 
		X1=X1<<8;
		X_out=X0+X1;   
	}
	Xg=(X_out/32.0)*1000;
	return (int)Xg;
}
int adxl345readY(){
	int Y0 = 0,Y1 = 0,Y_out = 0;
	double Yg;
	adxl345request(ADXL345_OUT_Y0, ADXL345_OUT_Y1);
	if(Wire.available()<=2){
		Y0 = Wire.read();
		Y1 = Wire.read(); 
		Y1=Y1<<8;
		Y_out=Y0+Y1;
	}
	Yg=(Y_out/32.0)*1000;
	return (int)Yg;
}
int adxl345readZ(){
	int Z1 = 0,Z0 = 0,Z_out = 0;
	double Zg;
	adxl345request(ADXL345_OUT_Z0, ADXL345_OUT_Z1);
	if(Wire.available()<=2){
		Z0 = Wire.read();
		Z1 = Wire.read(); 
		Z1=Z1<<8;
		Z_out=Z0+Z1;
	}
	Zg=(Z_out/32.0)*1000;
	return (int)Zg;
}
