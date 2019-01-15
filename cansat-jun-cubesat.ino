/* #---------------Cubesat-control---------------# *
 * |               v 0.1.0-bleeding              | *
 * |  A cansat-jun fork for SiriusSat            | *
 * |                                             | *
 * |    Hardware list:                           | *
 * |        atmega128                            | *
 * |        GY-801 IMU board                     | *
 * |        UART radio                           | *
 * |        SDCard reader                        | *
 * |        Thermal resistors                    | *
 * |        NEO-6M GPS                           | *
 * |        Light sensors                        | *
 * |                                             | *
 * #---------------------------------------------# */
 
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "l3g4200d.h"
#include "adxl345.h"
#include "hmc5883.h"
#include "bmp085.h"
#include "gps.h"

/* 
   DEFINES:
    SD_CS - sdcard SPI chip select pin (default: 4). Pulldown this pin to GND to set up the SD card. Must be disabled while operating. INT
    VREF - battery reference voltage (default: 4.5F). Used to determine battery charge level. This voltage is made by voltage divider. FLOAT
    VDIV - voltage divider coefficient (default: 2). DO NOT CHANGE FLOAT
    CALLSIGN - satellite callsign, should be as short as possible. STRING
*/

#define SD_CS 4
#define VREF 4.5F
#define VDIV 2
#define CALLSIGN "YKTSAT5"
#define VER "CubesatControl v0.1.0-bleeding"
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 8

/*
   LIBRARY SETUP
*/
L3G4200D gyro;
TinyGPSPlus gps;
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer, outsideThermometer;

unsigned short int timer = 0;
float baseAlt = 0;
String dataL[] = {"ET=", "T=", "PRS=", "VBAT=", "ALT=", "AX=", "AY=", "AZ=", "MX=", "MY=", "MZ=", "GX=", "GY=", "GZ=", "IT=", "OT="}; //Data labels

void formPacket(int * data, int start, int amount){
    String buffer0;
    for(int i = start; i < (start + amount); i++){ //5 variables at once
		buffer0 += dataL[i]; //Adding data labels
		buffer0 += String(*(data+i)); //Adding data
		buffer0 += " "; //Separator
	}
    sendData(buffer0);
}

void sendData(String data){
	data = String(CALLSIGN) + "[MAIN]: " + data + ";\n";
	Serial1.print(data);
	File dataFile = SD.open("yktsat5.log", FILE_WRITE);
	if (dataFile) {
		dataFile.println(data);
		dataFile.close();
	}
}

void sendgps(){
	Serial1.print(String(CALLSIGN)+ "[GPS]:"); printInt(gps.satellites.value(), gps.satellites.isValid(), 2); Serial1.print(F(" LAT=")); printFloat(gps.location.lat(), gps.location.isValid(), 11, 6); Serial1.print(F(" LON=")); printFloat(gps.location.lng(), gps.location.isValid(), 11, 6); Serial1.print(F(" SPD=")); printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2); Serial1.print(F(" DT=")); printDateTime(gps.date, gps.time);Serial1.println();
}
int ds18b20init(){
	int count = 0;
	sensors.begin();
	count = sensors.getDeviceCount();
	if (!sensors.getAddress(insideThermometer, 0)) return -1;
    if (!sensors.getAddress(outsideThermometer, 1)) return -2;
	sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);
	return count;
}
void ds18b20read(int *temp, DeviceAddress deviceAddress){
	*temp = (sensors.getTempC(deviceAddress))*10;
}

void setup(){
	delay(5000);
	Serial.begin(9600);
	Serial1.begin(9600);
	sendData("Running "+String(VER) + " compiled " + String(__TIMESTAMP__));
	sendData("WARMUP");
	
	Wire.begin();
	sendData("I2C OK");
	delay(50);
	
	gyro.enableDefault();
	delay(50);
	sendData("L3G OK");
	
	bmp085init();
	delay(50);
	float temp = 0;
	bmp085GetTemperature(bmp085ReadUT()); 
	temp = bmp085GetPressure(bmp085ReadUP());
	baseAlt = calcAltitude(temp);
	sendData("BMP OK");
	
	HMC5883init();
	delay(50);
	sendData("HMC OK");
	
	adxl345init();
	delay(50);
	sendData("ADXL OK");
	
	SD.begin(SD_CS);
	sendData("SD OK");
	delay(50);	
	
	ds18b20init();
	sendData("DS18 OK");
	delay(50);	
	
	sendData("INIT OK");
	delay(500);
	
}

void loop(){
	timer = millis();
	gyro.read();
	static int data[16];
	data[0] = millis()/1000;
	data[1] = bmp085GetTemperature(bmp085ReadUT()); 
	data[2] = bmp085GetPressure(bmp085ReadUP());
	data[3] = 0; 
	data[4] = calcAltitude(data[2]) - baseAlt;
	if (data[4] < 0) data[4] = 0;
	data[5] = adxl345readX();
	data[6] = adxl345readY();
	data[7] = adxl345readZ();
	data[8] = HMC5883readX();
	data[9] = HMC5883readY();
	data[10] = HMC5883readZ();
	data[11] = (int)gyro.g.x;
	data[12] = (int)gyro.g.y;
	data[13] = (int)gyro.g.z;
	ds18b20read(data+14, insideThermometer);
	ds18b20read(data+15, outsideThermometer);
    
    formPacket(data, 0, 5);
    formPacket(data, 5, 11);
    sendgps();
	delay(1000-(millis()-timer));
}

	
