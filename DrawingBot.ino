#include <ADNS2610.h>

#define OPTIMOUSE_SCLK 2                            // Serial clock pin on the Arduino
#define OPTIMOUSE_SDIO 3                            // Serial data (I/O) pin on the Arduino
#define MOTOR_RIGHT_FWD 6
#define MOTOR_RIGHT_BCK 5
#define MOTOR_LEFT_FWD 10
#define MOTOR_LEFT_BCK 9
#define STATE_IDLE 0
#define STATE_DRIVING 1
#define STATE_TURNING 2

int state;
int currentX, currentY; 
int targetX, targetY;

ADNS2610 optiMouse = ADNS2610(OPTIMOUSE_SCLK, OPTIMOUSE_SDIO);

void setup(){

	Serial.begin(9600);

	optiMouse.begin();
	optiMouse.setSleepEnabled(false);

	state = STATE_IDLE;
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

void loop(){
	updateMouse();

	switch(state){
		case STATE_IDLE:
			break;
		
		case STATE_DRIVING:
			if(currentY >= targetY){
				// stop motors
				setMotors(0,0);
				state = STATE_IDLE;
			} else {
				int deltaX = currentX - targetX;
				int correction = 0;
				Serial.println(deltaX);
				
				// super-simpel-regler
				correction = map(deltaX, -10, 10, -100, 100);
				setMotors(200 + correction, 200 - correction);
				
			}

			break;
		
		case STATE_TURNING:
			break;
	}

	if(state == STATE_IDLE){
		delay(500);
		drive(5000);
	}
}

void drive(int distance){
	state = STATE_DRIVING;
	targetY = currentY + distance;
	targetX = currentX;
}

void turn(int angle){
	// not implemented, yet
}

void updateMouse(){
  
  if (!optiMouse.verifyPID()) {
	Serial.println("fl");    
	Serial.flush();
	optiMouse.begin();
	return;  
  }  
  
  currentX += optiMouse.dy();
  currentY -= optiMouse.dx();
}
