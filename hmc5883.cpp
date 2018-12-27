#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "hmc5883.h"
void HMC5883init(){
	Wire.beginTransmission(HMC5883_ADDRESS); 
	Wire.write(HMC5883_CTRL_MODE);
	Wire.write(HMC5883_CTRL_MODEV);
	Wire.endTransmission();
}
void HMC5883request(byte r1){
	Wire.beginTransmission(HMC5883_ADDRESS);
	Wire.write(r1);
	Wire.endTransmission();
	Wire.requestFrom(HMC5883_ADDRESS, 2);
}
int HMC5883readX(){
	int x = 0;
	HMC5883request(HMC5883_OUT_X);
	if(2 <= Wire.available()){
		x = Wire.read()<<8;
		x |= Wire.read();
	}
	return x;
}
int HMC5883readY(){
	int y = 0;
	HMC5883request(HMC5883_OUT_Y);
	if(2 <= Wire.available()){
		y = Wire.read()<<8;
		y |= Wire.read();
	}
	return y;
}
int HMC5883readZ(){
	int z = 0;
	HMC5883request(HMC5883_OUT_Z);
	if(2 <= Wire.available()){
		z = Wire.read()<<8;
		z |= Wire.read();
	}
	return z;
}