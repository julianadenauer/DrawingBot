/*
 *
 * DrawingBot.ino
 *
 */


#define DEBUG

#define MOTOR_RIGHT_FWD	5
#define MOTOR_RIGHT_BCK	6
#define MOTOR_LEFT_FWD	9
#define MOTOR_LEFT_BCK	10
#define STATE_IDLE 0
#define STATE_DRIVING 1
#define STATE_TURNING 2
#define STATE_TURNING_WIDE 3
#define STATE_TURNING_RIGHT 4

#define ENC_0_A		2	//Interrupt0
#define ENC_0_B		4
#define ENC_1_A		3	//Interrupt1
#define ENC_1_B		7


const int encoderCountsPerTurn = 300;
const float wheelDiameter = 26.02;
const float wheelBase = 89;

const float mmToCounts = ((float)encoderCountsPerTurn) / (wheelDiameter * M_PI);



long lastX = 0;
long lastY = 0;

int state;
int programStep;

volatile long currentRight, currentLeft;
volatile long lastRight, lastLeft;
long deltaRight, deltaLeft;
long targetRight, targetLeft;

const int rampLength = 1000;
const int startPower = 100; // minimalwert bei der die Anfahrrampe beginnt

long startTime;

int leftMotorDir;
int rightMotorDir;


// var's for bresenham algorithmus used in turning-wide state
int bresenhamRatio;
long bresenhamCounter;

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
	#ifdef ADNS9800
		initADNS9800();
	#endif

	 // encoder 0 
	pinMode(ENC_0_A,		INPUT); 
	//digitalWrite(ENC_0_A,	HIGH);       // turn on pullup resistor
	pinMode(ENC_0_B,		INPUT); 
	//digitalWrite(ENC_0_B,	HIGH);       // turn on pullup resistor
	attachInterrupt(0, doEncoder0, CHANGE);  // encoder pin on interrupt 0 - pin 2
	// encoder 1
	pinMode(ENC_1_A,		INPUT);
	//digitalWrite(ENC_1_A,	HIGH);
	pinMode(ENC_1_B,		INPUT);
	//digitalWrite(ENC_1_B,	HIGH);
	attachInterrupt(1, doEncoder1, CHANGE);	// encoder pin on interrupt 1 - pin 3
 
	state = STATE_IDLE;
        programStep = 0;
}

void doEncoder0() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if(digitalRead(ENC_0_A) == digitalRead(ENC_0_B)) {
    currentRight--;
  } else {
    currentRight++;
  }
  //~ Serial.println(encoder0Pos,DEC);
}

void doEncoder1() {
  if(digitalRead(ENC_1_A) == digitalRead(ENC_1_B)) {
    currentLeft++;
  } else {
    currentLeft--;
  }
}


int updatePDControl(int delta)
{
    int correction = 0;
    // super-simpel-regler
    correction = constrain(delta * 3, -100, 100);
    iValue += delta;
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


void drive(float distance){
        
  
	state = STATE_DRIVING;
        if (distance >= 0)
        {
          leftMotorDir = 1;
          rightMotorDir = 1;
        }
        else
        {
          leftMotorDir = -1;
          rightMotorDir = -1;
        }

	targetRight = (long)(mmToCounts * distance);
	targetLeft = (long)(mmToCounts * distance);
        resetFilter();
        resetCounters();
}

/*
void turnRight(int ratio, int distance){
  resetCounters();
  targetRight = 0;
  bresenhamRatio = ratio;
  bresenhamCounter = 0;
  startTime = millis();
  resetFilter();
  
  targetLeft = distance;

  state = STATE_TURNING_RIGHT;
}
*/

void turnRight(float radius, float distance){
  resetCounters();

  targetRight = 0;
  bresenhamCounter = 0;

  if (distance >= 0)
  {
    leftMotorDir = 1;
    rightMotorDir = 1;
  }
  else
  {
    leftMotorDir = -1;
    rightMotorDir = -1;
  }
  
  float distanceLeft = distance * (radius + 0.5 * wheelBase) / radius;
  
  float speedRatio = (radius + 0.5 * wheelBase) / (radius - 0.5 * wheelBase);
  if (speedRatio < 0)
  {
    speedRatio =- speedRatio;
    leftMotorDir = -leftMotorDir;
  }
  
  targetLeft = (long)(distanceLeft * mmToCounts) * leftMotorDir;
 
  bresenhamRatio = (int)(speedRatio * 1000);
  
  startTime = millis();
  resetFilter();
 
  state = STATE_TURNING_RIGHT;
}


void resetCounters(){
  currentLeft = 0;
  currentRight = 0;
  lastLeft = 0;
  lastRight = 0;
}

///////////////////////////////////////
// loop
//////////////////////////////////////

void loop(){	
	
      deltaLeft = currentLeft - lastLeft;
      deltaRight = currentRight - lastRight;
      lastLeft = currentLeft;
      lastRight = currentRight;
      
	#ifdef DEBUG
		Serial.print("currentX:");
		Serial.print("\t\t");
		Serial.print(currentRight);
		Serial.print("\t\t");
		Serial.print("currentY:");
		Serial.print("\t\t");
		Serial.print(currentLeft);
		Serial.print("\t\t");
	#endif
    
	switch(state){
		case STATE_IDLE:
			break;
		
		case STATE_DRIVING:
			if(currentRight >= targetRight && currentLeft  >= targetLeft){
				// stop motors
				setMotors(0,0);
				state = STATE_IDLE;
			} else {
        // ENCODER VERSION
        long delta = currentRight - currentLeft; // calculate pos difference
        Serial.println(delta);
        int correction = updatePDControl(delta);
        setMotors(200 * rightMotorDir - correction, 200 * leftMotorDir + correction);
			}

			break;

    case STATE_TURNING_RIGHT:
      if(abs(currentLeft) >= abs(targetLeft)){
        setMotors(0,0);
        state = STATE_IDLE;          
      }
      else {
        bresenhamCounter += deltaLeft * 1000;       // hier *1000 damit wir das ratio genauer einstellen können
        while(bresenhamCounter > bresenhamRatio){
          targetRight += rightMotorDir;
          bresenhamCounter -= bresenhamRatio;  
        }
        
        int delta = targetRight - currentRight;
        int correction = updatePDControl(delta);

        int drivePower = 200;
        
        // anfahrrampe
        int dT = millis() - startTime;
        if(dT < rampLength){
            long temp = drivePower - startPower;
            temp *= dT;
            drivePower = startPower + (int) (temp / rampLength);
            Serial.println(drivePower);
        }

        setMotors(drivePower * rightMotorDir + correction, drivePower * leftMotorDir - correction);
      }
      break;
	}

	if(state == STATE_IDLE){
		delay(2000);
      switch (programStep) {
        case 0:
//          turnRight(2000, 20000);
//          turnRight(5000, 5810);  //5150
//          turnRight(67.0, 420.96);
            turnRight(390.0 / 2.0, 390.0 * M_PI);
//            drive(200);
          break;
        case 1:
//          drive(-200);
          // turn(90);
          break;
        case 2:
//          drive(400);
          break;
        case 3:
          // turn(90);
          break;
      }
      
      programStep = (programStep + 1) % 2;  // endless loop ...    
	}
}
