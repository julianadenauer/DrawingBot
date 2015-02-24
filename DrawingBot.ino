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
#define STATE_TURNING_LEFT 5
#define STATE_TURNING_ON_SPOT 6

#define ENC_0_A		2	//Interrupt0
#define ENC_0_B		4
#define ENC_1_A		3	//Interrupt1
#define ENC_1_B		7


const int encoderCountsPerTurn = 300;
const float wheelDiameter = 26.02;
const float wheelBase = 89;

const float mmToCounts = ((float)encoderCountsPerTurn) / (wheelDiameter * M_PI);

const int StopRampDist = 200;
const int MinDrivePower = 70;
const int MaxDrivePower = 200;

const int StartRampDuration = 500;

const int StopDistance = 10;

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
long bresenhamRatio;
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
        
        startTime = millis();
}


void turn(float radius, float distance)
{
  if (abs(radius) < 0.1)
  {
     turnOnSpot(distance * 360.0 / (2.0 * M_PI * radius));
     return;
  }
  
  resetFilter();
  resetCounters();

  targetRight = 0;
  targetLeft = 0;
  bresenhamCounter = 0;


  int turnSign = 1;
  if (radius < 0)
  {
     radius = -radius;
     turnSign = -1;
  }

  if (distance >= 0)
  {
    leftMotorDir = 1;
    rightMotorDir = 1;
  }
  else
  {
    distance = -distance;
    leftMotorDir = -1;
    rightMotorDir = -1;
  }
  
  float outerWheelDistance = distance * (radius + 0.5 * wheelBase) / radius;
  
  float speedRatio = (radius + 0.5 * wheelBase) / (radius - 0.5 * wheelBase); // hier kann div/0 auftreten, außerdem werte die den int (und sogar long) bereich übersteigen können
  if (speedRatio < 0.0)
  {
    speedRatio =- speedRatio;
    if (turnSign > 0) rightMotorDir = -rightMotorDir;
    else leftMotorDir = -leftMotorDir;
  }
  
  if (turnSign > 0)
  {
    targetLeft = (long)(outerWheelDistance * mmToCounts) * leftMotorDir;
    state = STATE_TURNING_RIGHT;
  }
  else
  {
    targetRight = (long)(outerWheelDistance * mmToCounts) * rightMotorDir;
    state = STATE_TURNING_LEFT;
  }
   
  bresenhamRatio = (long)(speedRatio * 1000.0);

/*
  Serial.print("speedRatio ");
  Serial.println(speedRatio);
  
  Serial.print("bresenhamRatio ");
  Serial.println(bresenhamRatio);
  
  Serial.print("targetLeft ");
  Serial.println(targetLeft);

  Serial.print("targetRight ");
  Serial.println(targetRight);
*/
  
  startTime = millis();
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

  startTime = millis();

  state = STATE_TURNING_ON_SPOT; 
}

void resetCounters(){
  currentLeft = 0;
  currentRight = 0;
  lastLeft = 0;
  lastRight = 0;
}


// ---------- start and stop ramp ----------
int getDrivePower(long distance)
{
  int drivePower = MaxDrivePower;
  
  if (distance < StopRampDist)
  {
    long temp = MaxDrivePower - MinDrivePower;
    temp *= distance;
    temp /= StopRampDist;
    temp += MinDrivePower;
    drivePower = (int)temp;
  }
  else
  {
    int dT = millis() - startTime;
    if(dT < StartRampDuration)
    {
        long temp = MaxDrivePower - MinDrivePower;
        temp *= dT;
        drivePower = MinDrivePower + (int) (temp / StartRampDuration);
    }
  }
  
  //Serial.println(drivePower);
  
  return drivePower;
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
           
           const int slowDownDist = 200;

           if (leftMotorDir > 0)  // (rightMotorDir == leftMotorDir)
             distance = min(targetRight - currentRight, targetLeft - currentLeft);
           else 
             distance = -max(targetRight - currentRight, targetLeft - currentLeft);
           
           int drivePower = getDrivePower(distance);
           
           if(distance < StopDistance)
           {
	      // stop motors
	      setMotors(0,0);
	      state = STATE_IDLE;
	   } else {
           
             // ENCODER VERSION
             long delta = currentRight - currentLeft; // calculate pos difference
             //Serial.println(delta);
             int correction = updatePDControl(delta);
             setMotors(drivePower * rightMotorDir - correction, drivePower * leftMotorDir + correction);
	   }
        }
	break;

    // ---------------------- state turning right -----------------------

        case STATE_TURNING_RIGHT:
        {
          long distance = targetLeft - currentLeft;
          if (targetLeft < 0) distance = -distance;
     
          if(distance < StopDistance){
            setMotors(0,0);
            state = STATE_IDLE;          
          }
          else 
          {
            bresenhamCounter += abs(deltaLeft * 1000);       // hier *1000 damit wir das ratio genauer einstellen können
            while(bresenhamCounter > bresenhamRatio)
            {
              targetRight += rightMotorDir;
              bresenhamCounter -= bresenhamRatio;  
            }
            
            int delta = targetRight - currentRight;
            
            int correction = updatePDControl(delta);
                
            int drivePower = getDrivePower(distance);

            setMotors(drivePower * rightMotorDir + correction, drivePower * leftMotorDir);  // vorher war correction auch auf linkem rad...versehen?
          }
        }
        break;


    // ---------------------- state turning left -----------------------

        case STATE_TURNING_LEFT:
        {
          long distance = targetRight - currentRight;
          if (targetRight < 0) distance = -distance;
          
          if(distance < StopDistance){
            setMotors(0,0);
            state = STATE_IDLE;
          }
          else 
          {
            bresenhamCounter += abs(deltaRight * 1000);       // hier *1000 damit wir das ratio genauer einstellen können
            while(bresenhamCounter > bresenhamRatio){
              targetLeft += leftMotorDir;
              bresenhamCounter -= bresenhamRatio;
            }
            
            int delta = targetLeft - currentLeft;
            int correction = updatePDControl(delta);
    
            int drivePower = getDrivePower(distance);
            
            setMotors(drivePower * rightMotorDir, drivePower * leftMotorDir + correction);
          }
        }
        break;

      
        // ------------------- state turning on spot --------------------
      
        case STATE_TURNING_ON_SPOT:
        {  
          long distance = min(abs(targetLeft - currentLeft), abs(targetRight - currentRight));
                 
          if (distance < StopDistance)
          {
            setMotors(0,0);
            state = STATE_IDLE;          
          }
          else
          {            
            int drivePower = getDrivePower(distance);

            int delta = currentLeft + currentRight;
            int correction = updatePDControl(delta);
            
            setMotors(drivePower * rightMotorDir - correction, drivePower * leftMotorDir - correction);
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
          turn(80, 80 * M_PI - 1);
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
