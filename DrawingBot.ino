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
#define STATE_TURNING_ON_SPOT 5

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

void turnOnSpot(float angleDeg)
{
  float distance = angleDeg * M_PI * wheelBase / 360.0;
  targetLeft = (long)(distance * mmToCounts);
  targetRight = -targetLeft;
  
  if (distance >= 0) leftMotorDir = 1;
  else leftMotorDir = -1;
  
  rightMotorDir = -leftMotorDir;

  resetCounters();
  resetFilter();

  state = STATE_TURNING_ON_SPOT;  
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

      // ---------------------- state driving -----------------------
		
	case STATE_DRIVING:
        {                
           long distance;

           int drivePower = 200;
           
           const int slowDownDist = 200;

           if (leftMotorDir > 0)  // (rightMotorDir == leftMotorDir)
             distance = min(targetRight - currentRight, targetLeft - currentLeft);
           else 
             distance = -max(targetRight - currentRight, targetLeft - currentLeft);
           
          // ramp down drive-power if near final angle
          if (distance < slowDownDist)
          {
            long temp = drivePower;
            temp *= distance;
            temp /= slowDownDist;
            temp += 70;  // min. drive-power
            drivePower = (int)temp;
          }
           
           
           if(distance < 10)
           {
	      // stop motors
	      setMotors(0,0);
	      state = STATE_IDLE;
	   } else {
           
             // ENCODER VERSION
             long delta = currentRight - currentLeft; // calculate pos difference
             Serial.println(delta);
             int correction = updatePDControl(delta);
             setMotors(drivePower * rightMotorDir - correction, drivePower * leftMotorDir + correction);
	   }
        }
	break;

    // ---------------------- state turning right -----------------------

        case STATE_TURNING_RIGHT:
        {
          if(abs(currentLeft) >= abs(targetLeft)){
            setMotors(0,0);
            state = STATE_IDLE;          
          }
          else 
          {
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
                //Serial.println(drivePower);
            }
    
            setMotors(drivePower * rightMotorDir + correction, drivePower * leftMotorDir - correction);
          }
        }
        break;
      
        // ------------------- state turning on spot --------------------
      
        case STATE_TURNING_ON_SPOT:
        {  
          long distance = min(abs(targetLeft - currentLeft), abs(targetRight - currentRight));
        
          int drivePower = 200;       
        
          const int slowDownDist = 200;
        
          // ramp down drive-power if near final angle
          if (distance < slowDownDist)
          {
            long temp = drivePower;
            temp *= distance;
            temp /= slowDownDist;
            temp += 70;  // min. drive-power
            drivePower = (int)temp;
          }
          
          if (distance < 10)
          {
            setMotors(0,0);
            state = STATE_IDLE;          
          }
          else
          {
            
            int delta = currentLeft + currentRight;
            int correction = updatePDControl(delta);
            
            setMotors(drivePower * rightMotorDir - correction, drivePower * leftMotorDir - correction);
            //setMotors(drivePower * rightMotorDir, drivePower * leftMotorDir);
          }
        }  
        break;
      
      }  // end of switch




	if(state == STATE_IDLE){
		if (programStep < 28) delay(500);
switch (programStep) {
        case 0:
          drive(150);
          break;
        case 1:
          turnOnSpot(-70);
          break;
        case 2:
          drive(60);
          break;
        case 3:
          turnOnSpot(140);
          break;
        case 4:
          drive(60);
          break;
        case 5:
          turnOnSpot(-70);  // 73
          break;
        case 6:
          drive(10);
          break;
        case 7:
          turnOnSpot(-90);
          break;
        case 8:
          drive(58);
          break;
        case 9:
          turnOnSpot(90);
          break;
        case 10:
          drive(35);
          break;
        case 11:
          turnOnSpot(90);
          break;
        case 12:
          drive(30);
          break;
        case 13:
          turnOnSpot(90);
          break;
        case 14:
          drive(25);
          break;
        case 15:
          turnOnSpot(-127);  // -130
          break;
        case 16:
          drive(37);
          break;
        case 17:
          turnOnSpot(-50);  // -50
          break;
        case 18:
          drive(40);
          break;
        case 19:
          turnOnSpot(90);
          break;
        case 20:
          drive(15);
          break;
        case 21:
          turnOnSpot(93);  // 90
          break;
        case 22:
          drive(156);
          break;
        case 23:
          turnOnSpot(90);
          break;
        case 24:
          turnRight(80, 80 * M_PI - 1);
          break;
        case 25:
          turnOnSpot(-90);
          break;                    
        case 26:
          drive(60);
          break;
        case 27:
          turnOnSpot(184);
          break;
        case 28:
          drive(-200);
          break;
        default:
          delay(1000);
      }

      // programStep = (programStep + 1) % 18;  // endless loop ...
        programStep += 1;
      }
}
