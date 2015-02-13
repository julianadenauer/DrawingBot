#define DEBUG

#define MOTOR_RIGHT_FWD 5
#define MOTOR_RIGHT_BCK 6
#define MOTOR_LEFT_FWD 9
#define MOTOR_LEFT_BCK 3 // 10
#define STATE_IDLE 0
#define STATE_DRIVING 1
#define STATE_TURNING 2
#define STATE_TURNING_WIDE 3

#include <SPI.h>
#include "ADNS9800.h"
const float sensorRadius = 58.0; // in mm
const float mouseCPI = 820.0;  // counts per inch of mousesensor

int state;
int programStep;
long currentX, currentY;
long currentDeltaX, currentDeltaY;
long targetX, targetY;

// var's for bresenham algorithmus used in turning-wide state
int bresenhamRatio;
int bresenhamCounter;

int iValue;

void  resetFilter(){
  iValue = 0;
}


////////////////////////////////
//setup
/////////////////////////////////
void setup(){
  
	Serial.begin(38400);
  
  //attachInterrupt(0, UpdatePointer, FALLING);
  initADNS9800();

	state = STATE_IDLE;
        programStep = 0;
}

int updatePDControl(int inVal)
{
    int correction = 0;				
    // super-simpel-regler
    correction = constrain(inVal * 3, -100, 100);
    iValue += inVal;
    correction += (iValue >> 5);  // (division by 32)
    
    return correction;
}


void setMotors(int right, int left){
	if(right>0){
		analogWrite(MOTOR_RIGHT_FWD, constrain(right, 0, 255));
		digitalWrite(MOTOR_RIGHT_BCK, LOW);
	}
	else{
		analogWrite(MOTOR_RIGHT_BCK, constrain(abs(right), 0, 255));
		digitalWrite(MOTOR_RIGHT_FWD, LOW);	
	}

	if(left>0){
		analogWrite(MOTOR_LEFT_FWD, constrain(left, 0, 255));
		digitalWrite(MOTOR_LEFT_BCK, LOW);
	}
	else{
		analogWrite(MOTOR_LEFT_BCK, constrain(abs(left), 0, 255));
		digitalWrite(MOTOR_LEFT_FWD, LOW);	
	}

}

void drive(int distance){
  float counts = distance * mouseCPI / 25.4f;
	state = STATE_DRIVING;
	targetY = currentY + counts;
	targetX = currentX;
  resetFilter();
}


void turn(int angle){
  state = STATE_TURNING;
	float counts = angle * sensorRadius * PI * mouseCPI / (180.0f * 25.4f);
  targetX = currentX + (int)counts;
  targetY = currentY;
  resetFilter();
}


void turnWide(int distance, int ratio){
  state = STATE_TURNING_WIDE;
	float counts = distance * mouseCPI / 25.4f;
	targetY = currentY + counts;
	targetX = currentX;
  bresenhamCounter = 0;
  bresenhamRatio = ratio;
  resetFilter();
}

///////////////////////////////////////
// loop
//////////////////////////////////////

void loop(){	
	
	if (1){
    UpdatePointer();
    currentX += *x;
    currentY += *y;
      
		#ifdef DEBUG
  		Serial.print("posx:");
  		Serial.print("\t\t");
  		Serial.print(currentX);
  		Serial.print("\t\t");
  		Serial.print("posy:");
  		Serial.print("\t\t");
  		Serial.print(currentY);
  		Serial.print("\t\t");
  		
  		Serial.print("dx:");
  		Serial.print("\t\t");
  		Serial.print(*x); //float (*x)/8200*25.4);
  		Serial.print("\t\t");
  		Serial.print("dy: ");
  		Serial.print("\t\t");
  		Serial.print(*y); //float (*y)/8200*25.4);
  		Serial.print("\t\t");
  		Serial.println();
  	#endif
  }
    
	switch(state){
		case STATE_IDLE:
			break;
		
		case STATE_DRIVING:
			if(currentY >= targetY){
				// stop motors
				setMotors(0,0);
				state = STATE_IDLE;
			} else {
        int deltaX = currentX - targetX;  // (eigentlich immer target - current, bei gelegenheit mal ändern)
        int correction = updatePDControl(deltaX);
				setMotors(200 - correction, 200 + correction);
			}

			break;
		
		case STATE_TURNING:
			if(currentX >= targetX){
				// stop motors
				setMotors(0,0);
				state = STATE_IDLE;
			} else {
        int deltaY = currentY - targetY;  // (eigentlich immer target - current, bei gelegenheit mal ändern)
        int correction = updatePDControl(deltaY);
				setMotors(200 - correction, -200 - correction); 
			}
			break;
		
      case STATE_TURNING_WIDE:
  			if(currentY >= targetY) {
  				// stop motors
  				setMotors(0,0);
  				state = STATE_IDLE;
  			} else {
          bresenhamCounter += currentDeltaY;
          while (bresenhamCounter > bresenhamRatio)
          {
            targetX++;
            bresenhamCounter -= bresenhamRatio;
          }
          int deltaX = currentX - targetX;  // (eigentlich immer target - current, bei gelegenheit mal ändern)
          int correction = updatePDControl(deltaX);
  				setMotors(200 - correction, 200 + correction);
  			}
			break;

	}

	if(state == STATE_IDLE){
		delay(2000);
      switch (programStep) {
        case 0:
          drive(600);
          break;
        case 1:
          turn(90);
          break;
        case 2:
          drive(400);
          break;
        case 3:
          turn(90);
          break;
      }
      
      programStep = (programStep + 1) % 4;  // endless loop ...    
	}
}