#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "hmc5883.h"
int HMC5883init(){
	Wire.beginTransmission(HMC5883_ADDRESS); 
	if(Wire.write(HMC5883_CTRL_MODE) == 0){
		return -1;
	}
	Wire.write(HMC5883_CTRL_MODEV);
	Wire.endTransmission();
	return 0;
}
int HMC5883request(byte r1){
	Wire.beginTransmission(HMC5883_ADDRESS);
	Wire.write(r1);
	Wire.endTransmission();
	if(Wire.requestFrom(HMC5883_ADDRESS, 2) == 0){
		return INT_MAX;
	}
	return 0;
}
int HMC5883readX(){
	int x = 0;
	if(HMC5883request(HMC5883_OUT_X) == INT_MAX){
		return INT_MAX;
	}
	if(2 <= Wire.available()){
		x = Wire.read()<<8;
		x |= Wire.read();
	}
	return x;
}
int HMC5883readY(){
	int y = 0;
	if(HMC5883request(HMC5883_OUT_Y) == INT_MAX){
		return INT_MAX;
	}
	if(2 <= Wire.available()){
		y = Wire.read()<<8;
		y |= Wire.read();
	}
	return y;
}
int HMC5883readZ(){
	int z = 0;
	if(HMC5883request(HMC5883_OUT_Y) == INT_MAX){
		return INT_MAX;
	}
	if(2 <= Wire.available()){
		z = Wire.read()<<8;
		z |= Wire.read();
	}
	return z;
}