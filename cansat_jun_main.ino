/* #------------CanSat-junior-control------------# *
 * |                v 1.2.1-staging              | *
 * |  Made by ThePetrovich for SJSA CanSat team  | *
 * |                                             | *
 * |    Hardware list:                           | *
 * |        atmega32u4                           | *
 * |        GY-801 IMU board                     | *
 * |        UART radio                           | *
 * |        a bootleg shift register             | *
 * |		someting i forgot                    | *
 * |                                             | *
 * #---------------------------------------------# */
 
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <L3G4200D.h>
#include <SoftwareSerial.h>
#include <Servo.h>

/* 
   DEFINES:
   1) Satellite configuration
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
   2) SPI-specific
    Register_ID - ADXL345 unique ID
	Register_2D - ADLX345 power control register
	Register_FORMAT - ADXL345 data resolution register. (default: 0x31). Set this to 0x03 to get 16g resolution. For more info, see ADXL345 datasheet.
	Register_X0 - ADXL345 X axis register #0 (default: 0x22). Used for I2C communication.
	Register_X1 - ADXL345 X axis register #1 (default: 0x33). Used for I2C communication.
	Register_Y0 - ADXL345 Y axis register #0 (default: 0x34). Used for I2C communication.
	Register_Y1 - ADXL345 Y axis register #1 (default: 0x35). Used for I2C communication.
	Register_Z0 - ADXL345 Z axis register #0 (default: 0x36). Used for I2C communication.
	Register_Z1 - ADXL345 Z axis register #1 (default: 0x37). Used for I2C communication.
	BMP085_ADDRESS - I2C BMP085 address (default: 0x77)
	HMC5883_ADDRESS - I2C HMC5883 address (default: 0x1E)
	ADXAddress - I2C ADXL345 address (default: 0x53)
*/
#define S_DATA 9
#define S_CLK 10
#define S_SCLK 11
#define S_MR 5
#define SD_CS 4
#define TX 5
#define RX 6
#define VREF 4.5F
#define VDIV 2
#define BAT A0
#define LED1 13
#define BUZZ A1
#define LRES A2
#define LVAL 450
#define SERVO 12
#define CALLSIGN "YKTSAT4"
#define ASL 101325.0F
#define VER "CJC v1.2.1-staging compiled 21.11.2018 23:55 YAKT"

#define Register_ID 0
#define Register_FORMAT 0x31
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37
#define BMP085_ADDRESS 0x77
#define HMC5883_ADDRESS 0x1E
#define ADXAddress 0x53
/*
   LIBRARY SETUP
*/
Servo myservo;
L3G4200D gyro;
SoftwareSerial ss(RX, TX);

//BMP calibration values, do not touch!
const unsigned char OSS = 0;
int ac1 = -1;
int ac2 = -1; 
int ac3 = -1; 
unsigned int ac4 = 0;
int b1 = 0; 
int b2 = 0;
int mb = 0;
long b5 = 0; 
//LED switch variable
volatile bool enabled = 1;
volatile bool deployed = 0;
int angle = 90;
bool detach = 0;
float baseAlt = 0;
int darkness = LVAL;
//I'm too lazy to use pointers or 2d char arrays, so i'll just make a global sting array
//:D
String dataL[] = {"ET=", "T=", "PRS=", "VBAT=", "ALT=", "AX=", "AY=", "AZ=", "MX=", "MY=", "MZ=", "GX=", "GY=", "GZ="}; //Data labels

/* Working with BMP085 using I2c */
void bmp085init(){
	//Reading calibration values from BMP
	ac1 = bmp085ReadInt(0xAA);
	ac2 = bmp085ReadInt(0xAC);
	ac3 = bmp085ReadInt(0xAE);
	ac4 = bmp085ReadInt(0xB0);
	b1 = bmp085ReadInt(0xB6);
	b2 = bmp085ReadInt(0xB8);
	mb = bmp085ReadInt(0xBA);	
}
short bmp085GetTemperature(unsigned int ut){
	//Making temperature correction using calibration values
    long x1, x2;
	int mc = bmp085ReadInt(0xBC);
	int md = bmp085ReadInt(0xBE);
	unsigned int ac5 = bmp085ReadInt(0xB2);
	unsigned int ac6 = bmp085ReadInt(0xB4);
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
    Wire.write(0xF4);
    Wire.write(0x2E);
    Wire.endTransmission();
    delay(5);
    ut = bmp085ReadInt(0xF6);
    return ut;
}
unsigned long bmp085ReadUP(){
    unsigned char msb, lsb, xlsb;
    unsigned long up = 0;
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(0xF4);
    Wire.write(0x34 + (OSS<<6));
    Wire.endTransmission();
    delay(2 + (3<<OSS));
    Wire.beginTransmission(BMP085_ADDRESS);
    Wire.write(0xF6);
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
/* Working with ADXL345 using I2c */
void adxl345init(){
	Wire.beginTransmission(ADXAddress);
	Wire.write(Register_2D);
	Wire.write(8);     
	Wire.endTransmission();	
	delay(50);
	Wire.beginTransmission(ADXAddress);
	Wire.write(Register_FORMAT);
	Wire.write(0x03);   	
	Wire.endTransmission();
}
void adxl345request(byte r1, byte r2){
	Wire.beginTransmission(ADXAddress); // transmit to device
	Wire.write(r1);
	Wire.write(r2);
	Wire.endTransmission();
	Wire.requestFrom(ADXAddress,2); 
}
int adxl345readX(){
	int X0 = 0,X1 = 0,X_out = 0;
	double Xg;
	adxl345request(Register_X0, Register_X1);
	if(Wire.available()<=2){
		X0 = Wire.read();
		X1 = Wire.read(); 
		X1=X1<<8;
		X_out=X0+X1;   
	}
	Xg=(X_out/32.0)*10;
	return (int)Xg;
}
int adxl345readY(){
	int Y0 = 0,Y1 = 0,Y_out = 0;
	double Yg;
	adxl345request(Register_Y0, Register_Y1);
	if(Wire.available()<=2){
		Y0 = Wire.read();
		Y1 = Wire.read(); 
		Y1=Y1<<8;
		Y_out=Y0+Y1;
	}
	Yg=(Y_out/32.0)*10;
	return (int)Yg;
}
int adxl345readZ(){
	int Z1 = 0,Z0 = 0,Z_out = 0;
	double Zg;
	adxl345request(Register_Z0, Register_Z1);
	if(Wire.available()<=2){
		Z0 = Wire.read();
		Z1 = Wire.read(); 
		Z1=Z1<<8;
		Z_out=Z0+Z1;
	}
	Zg=(Z_out/32.0);
	return (int)Zg;
}
/* Working with HMC5883 using I2c */
void HMC5883init(){
	Wire.beginTransmission(HMC5883_ADDRESS); 
	Wire.write(0x02);
	Wire.write(0x00);
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
	HMC5883request(0x03);
	if(2 <= Wire.available()){
		x = Wire.read()<<8;
		x |= Wire.read();
	}
	return x;
}
int HMC5883readY(){
	int y = 0;
	HMC5883request(0x05);
	if(2 <= Wire.available()){
		y = Wire.read()<<8;
		y |= Wire.read();
	}
	return y;
}
int HMC5883readZ(){
	int z = 0;
	HMC5883request(0x07);
	if(2 <= Wire.available()){
		z = Wire.read()<<8;
		z |= Wire.read();
	}
	return z;
}
/* Shift register code */
void shiftOut(byte data) {
	int i=0;
	int pinState;
	pinMode(S_CLK, OUTPUT);
	pinMode(S_DATA, OUTPUT);
	digitalWrite(S_DATA, 0);
	digitalWrite(S_CLK, 0);
	for (i=7; i>=0; i--){
		digitalWrite(S_CLK, 0);
		if (data & (1<<i)){
			pinState= 1;
		}
		else {  
			pinState= 0;
		}
		digitalWrite(S_DATA, pinState);
		digitalWrite(S_CLK, 1);
		digitalWrite(S_DATA, 0);
	}
	digitalWrite(S_CLK, 0);
}
/* LED blink */
void blink(){
	if (!enabled){
		digitalWrite(LED1, HIGH);
		enabled = true;
	}
	else{
		digitalWrite(LED1, LOW);
		enabled = false;
	}	
}
void formPacket(int * data, int start, int amount){
    string buffer0;
    for(int = start; i < (start + amount); i++){ //5 variables at once
		buffer0 += dataL[i]; //Adding data labels
		buffer0 += String(*(data+i)); //Adding data
		buffer0 += " "; //Separator
	}
    sendData(buffer0);
}
/* Function for data transmission */
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
	sendData("Running "+String(VER));
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
void handleIndicators(){
    const static byte a[9] = {B00000000, B00000010, B00000110, B00001110, B00011110, B00111110, B01111110, B11111110, B11111111}; //Shift register values array
	digitalWrite(S_SCLK, 0); 
	int n = int(analogRead(BAT)/128); //Trying to 'guess' battery voltage
	if (n > 8) n = 8; //We have 8 bits on S/R, so the value shoud not exceed 8
	shiftOut(a[n]); //Sending out battery level indication
	digitalWrite(S_SCLK, 1); 
}

void loop(){
	gyro.read();
	static int data[14];
	//String buffer0;
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
/* Timer interrupt handler */

	
