#include <Arduino.h>
#include "indication.h"
volatile bool enabled = true;

Matrix8 matrix(11, 10, 9);

void handleIndicators(){
    //const static byte a[9] = {B00000000, B00000010, B00000110, B00001110, B00011110, B00111110, B01111110, B11111110, B11111111}; //Shift register values array
	//digitalWrite(S_SCLK, 0); 
	int n = int(analogRead(BAT)/100); //Trying to 'guess' battery voltage
  static int last_n;
	if (n > 8) n = 8; //We have 8 bits on S/R, so the value shoud not exceed 8
  if(last_n != n){
    matrix.clear();
  }
	for(int x = 0; x < 8; x++){
		for(int y = 0; y < n; y++){
			matrix.set(x, y, 1);
		}
	}
	matrix.draw();
  last_n = n;
	//shiftOut(a[n]); //Sending out battery level indication
	//digitalWrite(S_SCLK, 1); 
}
/*void shiftOut(byte data) {
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
}*/
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
