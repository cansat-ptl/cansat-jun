/* #------------CanSat-junior-control------------# *
 * |               v 1.5.6-bleeding              | *
 * |  Made by ThePetrovich for SJSA CanSat team  | *
 * |                                             | *
 * |    Hardware list:                           | *
 * |        atmega32u4                           | *
 * |        GY-801 IMU board                     | *
 * |        UART radio                           | *
 * |        a bootleg shift register             | *
 * |        something i forgot                   | *
 * |                                             | *
 * #---------------------------------------------# */
 
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include "l3g4200d.h"
#include "adxl345.h"
#include "hmc5883.h"
#include "bmp085.h"
#include "indication.h"

/* 
   DEFINES:
    S_DATA - shift register serial data interface pin (default: 9). Used for S/R data transmission. INT
    S_CLK - shift register clock pin (default: 11). Used for S/R data transmission. INT
    S_SCKL - shift register refresh pin (default: 10). Pulldown this to GND to reset the shift register, must be done after each data transfer. INT
    SD_CS - sdcard SPI chip select pin (default: 4). Pulldown this pin to GND to set up the SD card. Must be disabled while operating. INT
    TX_CS - radio transmitter enable pin (default: 8). Pulldown this pin to GND to do i don't know what. Must be disabled during operation. INT
    TX_SET - radio transmitter setup pin (default: 7). Pulldown this pin to GND to use AT commands. Must be disabled during operation. INT
    VREF - battery reference voltage (default: 4.5F). Used to determine battery charge level. This voltage is made by voltage divider. FLOAT
    VDIV - voltage divider coefficient (default: 2). DO NOT CHANGE FLOAT
    BAT - battery voltage measurement pin (default: A0). INT
    LED1 - something led pin (default: 12). Indicates something. INT
    BUZZ - search buzzer pin (default: A1). Helps you to find your cansat by emitting sound. INT
    LRES - light sensor pin (default: A2). Used to determine if cansat was detached from the rocket. INT
    SERVO - servomotor pin (default: 13). INT
    CALLSIGN - satellite callsign, should be as short as possible. STRING
    ASL - pressure at sea level (in Pa)
*/

#define SD_CS 4
#define TX 5
#define RX 6
#define VREF 4.5F
#define VDIV 2
#define BUZZ A1
#define LRES A2
#define LVAL 450
#define SERVO 12
#define CALLSIGN "YKTSAT4"
#define VER "CJC v1.5.6-bleeding"

/*
   LIBRARY SETUP
*/
Servo myservo;
L3G4200D gyro;
SoftwareSerial ss(RX, TX);

volatile bool deployed = 0;
int angle = 90;
bool detach = 0;
float baseAlt = 0;
int darkness = LVAL;
String dataL[] = {"ET=", "T=", "PRS=", "VBAT=", "ALT=", "AX=", "AY=", "AZ=", "MX=", "MY=", "MZ=", "GX=", "GY=", "GZ="}; //Data labels

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
	data = String(CALLSIGN) + ": " + data + ";\n";
	Serial.print(data);
	ss.print(data);
	File dataFile = SD.open("yktsat4.log", FILE_WRITE);
	if (dataFile) {
		dataFile.println(data);
		dataFile.close();
	}
}

void setup(){
	delay(5000);
	Serial.begin(9600);
	ss.begin(9600);
	sendData("Running "+String(VER) + " compiled " + String(__TIMESTAMP__));
	sendData("WARMUP");
	
	myservo.attach(SERVO);
	myservo.write(5);
	myservo.detach();
	delay(250);
	sendData("SERVO OK");
	darkness = analogRead(LRES);
	while (analogRead(LRES)+50 >= darkness){
		sendData("WAITING");
		delay(1000);
	}
	sendData("LOCK");
	myservo.attach(SERVO);
	myservo.write(90);
	delay(2000);
	myservo.detach();
	
	Wire.begin();
	sendData("I2C OK");
	gyro.enableDefault();
	delay(50);
	sendData("L3G OK");
	
	bmp085init();
	float temp = 0;
	bmp085GetTemperature(bmp085ReadUT()); 
	temp = bmp085GetPressure(bmp085ReadUP());
	baseAlt = calcAltitude(temp);
	delay(50);
	sendData("BMP OK");
	
	HMC5883init();
	delay(50);
	sendData("HMC OK");
	
	adxl345init();
	delay(50);
	sendData("ADXL OK");
	delay(1000);
	
	pinMode(S_SCLK, OUTPUT);
	pinMode(LED1, OUTPUT);
	sendData("PINMODE OK");
	
	digitalWrite(S_SCLK, 0);
	shiftOut(B00000000);
	digitalWrite(S_SCLK, 1);
	sendData("SR OK");
	delay(1000);
	
	SD.begin(SD_CS);
	sendData("SD OK");
	delay(250);	
	
	sendData("INIT OK");
	delay(2000);
	
}
void handleServo(){
    int light = 0;
	light = analogRead(LRES);
	if (detach && !deployed){
		myservo.attach(SERVO);
		angle = 5;
		myservo.write(angle);
		delay(500);
		myservo.detach();
		sendData("DEPLOY " + String(millis()/1000));
		deployed = true;
	}
	if(light >= darkness-50 && !deployed){
		sendData("DETACH " + String(millis()/1000));
		detach = true;
		delay(500);
	}
}

void loop(){
	gyro.read();
	static int data[14];
	data[0] = millis()/1000;
	data[1] = bmp085GetTemperature(bmp085ReadUT()); 
	data[2] = bmp085GetPressure(bmp085ReadUP());
	data[3] = analogRead(BAT); 
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
    
    formPacket(data, 0, 5);
    formPacket(data, 5, 9);
    
    handleServo();
    handleIndicators();
    
	delay(750);
	blink();
}

	
