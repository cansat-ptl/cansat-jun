/* #---------------Cubesat-control---------------# *
 * |               v 0.3.1-bleeding              | *
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
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "pid.h"
#include "l3g4200d.h"
#include "adxl345.h"
#include "hmc5883.h"
#include "bmp085.h"
#include "gps.h"

/* 
   DEFINES:
    SD_CS - sdcard SPI chip select pin (default: 35). Pulldown this pin to GND to set up the SD card. Must be disabled while operating. INT
    VREF - battery reference voltage (default: 4.5F). Used to determine battery charge level. This voltage is made by voltage divider. FLOAT
    VDIV - voltage divider coefficient (default: 2). DO NOT CHANGE FLOAT
    CALLSIGN - satellite callsign, should be as short as possible. STRING
	ONEWIREPIN - one wire bus pin (default: 36)
	TEMPERATURE_PRECISION - DS28B20 measurement resolution in bits (default: 8)
	RWHEELPIN - reaction wheel control pin (default: 34)
	TARGET - camera point angle in degrees(default: 90)
*/

#define SD_CS 35
#define VREF 4.5F
#define VDIV 2
#define CALLSIGN "YKTSAT5"
#define VER "CubesatControl v0.3.1-bleeding"
#define ONEWIREPIN 36
#define TEMPERATURE_PRECISION 8
#define RWHEELPIN 34
#define TARGET 90

#ifndef INT_MAX
#define INT_MAX 32767
#endif
/*
   LIBRARY SETUP
*/
L3G4200D gyro;
TinyGPSPlus gps;
OneWire oneWire(ONEWIREPIN); 
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer, outsideThermometer;
Servo rwheel;

double Kp=200,Ki=1.5,Kd=0.75;
double Setpoint=TARGET,rpm=10000,temp=0;
PID myPID(&temp,&rpm,&Setpoint,Kp,Ki,Kd,DIRECT);
bool actives[4] = {false, false, false, false};

unsigned short int timer = 20;
float baseAlt = 0;
String dataL[] = {"ET=", "GX=", "GY=", "GZ=", "VBAT=", "AX=", "AY=", "AZ=", "MX=", "MY=", "MZ=", "ANGL=", "RPM=", "DBG=", "RSVD=", "RSVD="}; //Data labels

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
	//File dataFile = SD.open("yktsat5.log", FILE_WRITE);
	//if (dataFile) {
		//dataFile.println(data);
		//dataFile.close();
	//}
}

void sendgps(){
	Serial1.print(String(CALLSIGN)+ "[GPS]:"); 
	Serial1.print(F(" LAT=")); 
	printFloat(gps.location.lat(), gps.location.isValid(), 11, 6); 
	Serial1.print(F(" LON=")); 
	printFloat(gps.location.lng(), gps.location.isValid(), 11, 6); 
	Serial1.print(F(" DT=")); 
	printDateTime(gps.date, gps.time);
	Serial1.println(";");
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

double rw_setspeed(int a, int b){
    a*=(-1);
    temp = atan2((float)a,(float)b)/(3.1415F)*180.0F;
    if (temp < (Setpoint - 180))
        temp += 360;
    else if (temp > (Setpoint + 180))
        temp -= 360;
    myPID.Compute();         // computing current RPM 
    return rpm;
}

void setup(){
	pinMode(51, OUTPUT);
	digitalWrite(51, HIGH);
	delay(5000);
	Serial.begin(9600);
	Serial1.begin(9600);
	sendData("Running "+String(VER) + " compiled " + String(__TIMESTAMP__));
	sendData("WARMUP");
	
	Wire.begin();
	sendData("I2C OK");
	delay(50);
	
	if(gyro.enableDefault() == 0){
		sendData(F("L3G OK"));
		actives[0] = true;
	}
	else sendData(F("L3G ERR"));
	delay(50);
	
	//bmp085init();
	delay(50);
	//float temp = 0;
	//bmp085GetTemperature(bmp085ReadUT()); 
	//temp = bmp085GetPressure(bmp085ReadUP());
	//baseAlt = calcAltitude(temp);
	sendData(F("BMP DISABLED"));
	
	if(HMC5883init() == 0){
	    sendData(F("HMC OK"));
		actives[2] = true;
	}
	else sendData(F("HMC ERR"));
	delay(50);
	
	if(adxl345init() == 0){
	    sendData(F("ADXL OK"));
		actives[3] = true;
	}
	else sendData(F("ADXL ERR"));
	delay(50);
	
	rwheel.attach(RWHEELPIN);
	rwheel.write(0);
	delay(1000);
	sendData("RW OK");
	
	//SD.begin(SD_CS);
	//sendData("SD OK");
	//delay(50);	
	
	//ds18b20init();
	//sendData("DS18 OK");
	//delay(50);	
	myPID.SetOutputLimits(0, 20000);
    myPID.SetMode(AUTOMATIC);
  
	sendData("INIT OK");
	digitalWrite(51, LOW);
	delay(500);
}
bool led = false;
void loop(){
	int x = HMC5883readX();
	int z = HMC5883readZ();
	if (timer == 20){
	    gyro.read();
		timer = 0;
	    static int data[16];
	    data[0] = millis()/1000;
		if(actives[0]){
	        data[1] = (int)gyro.g.x;
	        data[2] = (int)gyro.g.y;
	        data[3] = (int)gyro.g.z;
		}
		else {
			data[1] = INT_MAX;
			data[2] = INT_MAX;
			data[3] = INT_MAX;
		}
	    data[4] = 0;
	    //if (data[4] < 0) data[4] = 0;
		if(actives[3]){
	        data[5] = adxl345readX();
	        data[6] = adxl345readY();
	        data[7] = adxl345readZ();
		}
		else {
			data[5] = INT_MAX;
			data[6] = INT_MAX;
			data[7] = INT_MAX;
		}
		if(actives[2]){
	        data[8] = HMC5883readX();
	        data[9] = HMC5883readY();
	        data[10] = HMC5883readZ();
		}
		else {
			data[8] = INT_MAX;
			data[9] = INT_MAX;
			data[10] = INT_MAX;
		}
	    data[11] = temp; 
	    data[12] = rpm;
	    data[13] = timer;
	    data[14] = 0;
	    data[15] = 0;
	
	    //ds18b20read(data+14, insideThermometer);
	    //ds18b20read(data+15, outsideThermometer);
        
        formPacket(data, 0, 5);
        formPacket(data, 5, 11);
        sendgps();
	    
	}
	if (timer % 10 == 0){
		if (led){
	        digitalWrite(51,LOW);
			led = false;
		}	
		else {
			digitalWrite(51, HIGH);
			led = true;
		}
	}
	timer++;
	delay(50);
	rpm = rw_setspeed(z, x);
	rwheel.write(map((int)rpm, 0, 20000, 0, 120));
}

	
