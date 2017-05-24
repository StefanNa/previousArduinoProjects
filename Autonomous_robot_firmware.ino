/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  

Title: Robot firmware for MOE2 competition
Author : MOE2, Group 2
Description : Program created specifically for P2 project. Controls a diferential drive robot powered by DC motors. 
Implemented features such as trajectory plant, position calcualtion via rotary encoders, PID feedback control, 
obsticle detection and avoidance.
Created : May, 2017
Modified : 14/05/2017
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/
  
/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::DEFINITIONS:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/

  //pin definitions motor 1 and 2. Connected to digital pins capable of hardware PWM mdoe. 
  #define bridge1APin  6 // H-bridge leg 1
  #define bridge2APin  5 // H-bridge leg 2
  #define bridge3APin  9 
  #define bridge4APin  11
  
  //pins for encoders on motor 1 and 2
  #define encoder1A  2 // interrupt is on pin 2
  #define encoder1B  4 // just digital input
  #define encoder2A  3 
  #define encoder2B  10 

  //Ultrasonic sensor definitions (3 sensors total)
  #define trigPin 12 
  #define echoPin 13
  #define trigPinR 8
  #define echoPinR 7
  #define trigPinL A5
  #define echoPinL A4
  #define distancelimit 17 // change for distance in cm

/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::VARIABLE DECLARATION:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/
  
  // Speed as PWM output. Value for motor speed should be 0-255.
  byte M1speedInitialF = 150; // initial base speed for driving forward. "F" stands for faster. This value is unchanged.
  byte M2speedInitialF = 120;
  byte M1speedInitialS = 140; // initial base speed for rotating. "S" stands for slower.
  byte M2speedInitialS = 115; 
  byte M1speed = M1speedInitialF; // This variable is constantly being adjusted in PID controler
  byte M2speed = M2speedInitialF;
    
  //initializing variables for the US sensors
  long duration, durationL, durationR, distance, distanceL, distanceR;
  bool stopFlagM = 0;
  bool stopFlagL= 0;
  bool stopFlagR = 0;  
  bool stopFlag = 0;
  bool resumeFlag = false;
  unsigned long stopped;
  unsigned long standing;
  int waitingTime =2000;
  int box = 25; // change for different obsticle size
  
  //initialize encoder counters
  volatile int countM1 = 0;
  volatile int countM2 = 0;

  //initialize coordinate system variables
  float omega0 = PI/2; // change to adjust initial angle to origin axis. 
  float x = 413;  // initial position coordinates
  float y = 20;
  float R = 9.4/2; // wheel radius
  float D = 19.5; // axis distance between wheels
  float Nl = 1120; // encoder counts per full rotation left wheel
  float Nr = 1117; 
  float nl,nr; // real time counts from the encoder
  float stepl, stepr; // distance per encoder count in cm
  float RICR; // Radius of Instantanious Center of Rotation
  float ICR[1]; // array for storing ICR coordinates x and y
  float omega;
  float xyomega[2]; // array to store returned real position coordinates x, y, and angle
  int CP =0; // pointer to xyomega array
  float x1; // for remembering coordinates next loop
  float y1;
  float omega1;
  float refPos;
  float realPos;

  
  //initialize trajectory variables
  float posVector[] = {30.00, 297.00, PI, 246.50, 3*PI/2, 50.00, 5*PI/2, 291.00, 3*PI, 82.00, 7*PI/2, 50.00}; // change to plant trajectory. Each value is a waypoint robot will reach.  
                     // - ,     y     o     x       o       y       o     y       o     x       o       y
  float PIDvector[]= {413.00, 297.00, 246.50, 291.00, 82.00}; // change to redifine reference for PID controler. 
                     // x       y       x       y       x
 
  bool checkPoint0 = true; // true for start condition
  bool checkPoint1 = false;
  bool checkPoint2 = false;
  bool checkPoint3 = false;
  bool checkPoint4 = false;
  bool checkPoint5 = false;
  bool checkPoint6 = false;
  bool checkPoint7 = false;
  bool checkPoint8 = false;
  bool checkPoint9 = false;
  bool checkPoint10 = false;
  bool checkPoint11 = false;   
  bool axisUp = true;


  //PID variables. Change to tune K values of PID
  float kp = 5.00; // K proportional
  float ki = 0.003;  // K integral
  float kd = 10000.00;  // K derivative
  float error, errSum, lastErr, dErr; // variables for computing error
  float output;
  unsigned long interval, currentTime; // for calculating time bvetween error measurments
  unsigned long previousTime =0;
  
  // MISC 
  bool driveFW = 1; // to diferentiate if we driwing forward or rotating
  int count = 0; // counter for periodically computing PID and coordinates
  float oldRef; // saving PID reference for returning after obsticle avoidance

/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::INTERUPT SERVICE ROUTINES:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/

/*
 * Summary:      ISR to handle encoder interrupts on motor 1. 
 *               Counts up if rotating clockwise, and counts down if in reverse. 
 * Parameters:   N/A
 * Return:       countM1.
 */
void counterM1()
{
    //check for encoder phases, determine direction
    if (digitalRead(encoder1A) == digitalRead(encoder1B)) 
    {
        countM1--;
    } 
    else 
    {
        countM1++; 
    }
}

/*
 * Summary:      ISR to handle encoder interrupts on motor 2. 
 *               Counts up if rotating counter-clockwise, and counts down if in reverse. 
 * Parameters:   N/A
 * Return:       countM2.
 */
void counterM2()
{ 
    if (digitalRead(encoder2A) == digitalRead(encoder2B)) 
    {
        countM2++; // its reverse because motor is on another side
    } 
    else 
    {
        countM2--;
    }
}

/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::MOTOR CONTROL FUNCTIONS:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/

/*
 * Summary:      Drive motor 1 in forward direction. 
 * Parameters:   PWM value for speed.  
 * Return:       N/A
 */
void M1forward(byte M1speed)
{
  digitalWrite(bridge2APin, LOW);
  analogWrite(bridge1APin, M1speed);
}

/*
 * Summary:      Drive motor 1 in reverse direction. 
 * Parameters:   PWM value for speed.  
 * Return:       N/A
 */
void M1backward(byte M1speed)
{
  digitalWrite(bridge1APin, LOW);
  analogWrite(bridge2APin, M1speed);
}

/*
 * Summary:      Drive motor 2 in forward direction. 
 *               Note reverse in H bridge since motors 
 *               are mirrored position of each other.
 * Parameters:   PWM value for speed.  
 * Return:       N/A
 */
void M2forward(byte M2speed)
{
  digitalWrite(bridge3APin, LOW);
  analogWrite(bridge4APin, M2speed);
}

/*
 * Summary:      Drive motor 2 in reverse direction. 
 * Parameters:   PWM value for speed.  
 * Return:       N/A
 */
void M2backward(byte M2speed)
{
  digitalWrite(bridge4APin, LOW);
  analogWrite(bridge3APin, M2speed);
  
}

/*
 * Summary:      Stop both motors. 
 * Parameters:   N/A
 * Return:       N/A
 */
void stopMotors()
{
  digitalWrite(bridge1APin, LOW); 
  digitalWrite(bridge2APin, LOW); 
  digitalWrite(bridge3APin, LOW); 
  digitalWrite(bridge4APin, LOW); 
}

/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::DRIVING TASKS:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/

/*
 * Summary:      Drives the robot forward. 
 *               Control to constantly start at the base speed.
 *               Sets flag true for driveFW so it can be chacked in adjustSpeed() function.
 * Parameters:   N/A
 * Return:       bool driveFW.
 */
void driveForward() 
{
    // put motor 1 and 2 in forward motion
    M1speed = M1speedInitialF;
    M2speed = M2speedInitialF;
    M1forward(M1speed);
    M2forward(M2speed);
    driveFW = 1;
}

/*
 * Summary:      Rotates the robot counter-clockwise. Delay for de-acceleration. 
 *               Control to constantly start at the base speed.
 *               Sets flag false for driveFW so it can be chacked in adjustSpeed() function.
 * Parameters:   N/A
 * Return:       bool driveFW.
 */
void rotateLeft() 
{
    delay(500);
    previousTime=millis()+1; // if it was zero, the error could not be computed in PID after turning to avoid obsticle
    M1speed=M1speedInitialS;
    M2speed=M2speedInitialS;
    M1forward(M1speed);
    M2backward(M2speed);
    driveFW = 0;
}

/*
 * Summary:      Rotates the robot clockwise. Delay for de-acceleration. 
 *               Control to constantly start at the base speed.
 *               Sets flag false for driveFW so it can be chacked in adjustSpeed() function.
 * Parameters:   N/A
 * Return:       bool driveFW.
 */
void rotateRight() 
{
    delay(500);
    previousTime=millis()-1;
    M1speed=M1speedInitialS;
    M2speed=M2speedInitialS;
    M1backward(M1speed);
    M2forward(M2speed);
    driveFW = 0;
}

/*
 * Summary:      Determines weather robot moving forward so it would adjust the correct base speed.
 *               Determiens weather it is moving up the axis (error correction holds inverse relationship). 
 * Parameters:   Output value from the PID controller.
 * Return:       N/A.
 */
void adjustSpeed(byte output)
{
       if (driveFW == 1) 
   {
        if (axisUp == true)
        {
          M1speed = - output + M1speedInitialF;   
          M2speed = + output + M2speedInitialF;
        }
        else
        {
          M1speed = + output + M1speedInitialF;   
          M2speed = - output + M2speedInitialF;
        }
   M1forward(M1speed);
   M2forward(M2speed);
   }
   else
   {
    M1speed = - output + M1speedInitialS;
    M1forward(M1speed);
    
    M2speed = + output + M2speedInitialS;
    M2backward(M2speed);
   }
}

/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::FEEDBACK CONTROLLER:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  

/*
 * Summary:      Computes PID. Calculates an error based on how much robot deviates from reference coordinate. 
 *               Reference coordinate corespond to coordinate of an axis robot is intended to move on in a straight line. 
 *               Modeled by an example from the link below, output fitted for PWM adjustment. 
 *               http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * Parameters:   Reference coordinate, Real position coordinate. 
 * Return:       output.
 */
float computePID( float refPos, float realPos)
{
   currentTime = millis();
   interval = (currentTime - previousTime);
   realPos = xyomega[CP];
   error = refPos - realPos;
   errSum += (error * interval);
   dErr = (error - lastErr) / interval;
   
   output = kp * error+ ki * errSum+ dErr*kd;
   output = constrain(output, -50, 50);
   output = round(output);
   adjustSpeed(output);
   
   lastErr = error;
   previousTime=currentTime;
}

/*
 * Summary:      Computes real time position of the robot in coordinate system of the obsticle track. 
 *               Converts encoder ticks to real life distance in cm.  
 *               Cannot be computed if both wheels rotated same amount of encoder ticks!! 
 * Parameters:   R, D, Nl, Nr, nl, nr, x, y, omega0. 
 * Return:       xyomega[] populated with real position values.
 */
void xyPosition(float R, float D, float Nl,float Nr,int nl,int nr, float x, float y, float omega0)
{

  
  stepl=2*PI*R/Nl;
  stepr=2*PI*R/Nr;

  if (nr*stepr != nl*stepl)
  {

        RICR=(D/2) *(nl*stepl+nr*stepr)/(nr*stepr-nl*stepl);
        ICR[0]={x-RICR*sin(omega0)};
        ICR[1]={y+RICR*cos(omega0)};
        omega=(nr*stepr-nl*stepl)/D;
    
    x1=cos(omega)*(x-ICR[0])-sin(omega)*(y-ICR[1])+ICR[0];
    y1=sin(omega)*(x-ICR[0])+cos(omega)*(y-ICR[1])+ICR[1];
    omega1=(nr*stepr-nl*stepl)/D + omega0;
    }
  
     else
     {
        x1=x+cos(omega0)*(nr*stepr+nl*stepr)/2;
        y1=y+sin(omega0)*(nr*stepr+nl*stepr)/2;
        omega1=omega0;
     }

     
      xyomega[0]= {x1};
      xyomega[1]= {y1};
      xyomega[2]= {omega1};

      

      countM1 = countM1 - nr;
      countM2 = countM2 - nl;
}

/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::OBSTICLE AVOIDANCE TASKS:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/

/*
 * Summary:      Reads middle ultrasonic sensor and calculates distance to an obsticle.
 *               Tells weather it detected an obsticle within defined distance.
 * Parameters:   N/A
 * Return:       bool stopFlagM.
 */
int sensor_readM() 
{
    digitalWrite(trigPin, LOW); // making sure trigpin is low
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); // sends a signal on trigger 10 microseconds long
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW);   // sets the trigpin to low again  
    duration = pulseIn(echoPin, HIGH); // "listens" how long until echo signal is received. timout is for how long the pulsein will wait for signal. it takes 500 microseconds for sound to travel 20 cm. 
    distance = (duration/2) / 29.1;     // formula for calculating distance in relation to the speed of sound
   
    if (distance < distancelimit)
    {
        stopFlagM = 1;
    }
    else 
    {
        stopFlagM = 0;
    }
    return stopFlagM;
}

/*
 * Summary:      Reads left ultrasonic sensor and calculates distance to an obsticle.
 *               Tells weather it detected an obsticle within defined distance.
 * Parameters:   N/A
 * Return:       bool stopFlagL.
 */
int sensor_readL() 
{
    digitalWrite(trigPinL, LOW); 
    delayMicroseconds(2);   
    digitalWrite(trigPinL, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPinL, LOW); 
    duration = pulseIn(echoPinL, HIGH);
    distance = (duration/2) / 29.1; 
   
    if (distance < distancelimit) 
    {
        stopFlagL = 1;
    }
    else 
    {
        stopFlagL = 0;
    }
    return stopFlagL;
}

/*
 * Summary:      Reads right ultrasonic sensor and calculates distance to an obsticle.
 *               Tells weather it detected an obsticle within defined distance.
 * Parameters:   N/A
 * Return:       bool stopFlagR.
 */
int sensor_readR() 
{
    digitalWrite(trigPinR, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPinR, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPinR, LOW); 
    duration = pulseIn(echoPinR, HIGH);
    distance = (duration/2) / 29.1; 
   
    if (distance < distancelimit) 
    {
        stopFlagR = 1;
    }
    else 
    {
        stopFlagR = 0;
    }
    return stopFlagR;
}

/*
 * Summary:      Reads all ultrasonic sensors.
 *               If any of the sensors returns true for stopflag, turns the global stopflag true.
 * Parameters:   N/A
 * Return:       bool stopFlag.
 */
int sensor_read() 
{
  sensor_readM();
  sensor_readL(); 
  sensor_readR(); 

  if (stopFlagM == true ||stopFlagL == true ||stopFlagR == true)
    {
      stopFlag = true;
    }
  else 
    {
      stopFlag = false;
    }
}

/*
 * Summary:      Procedure to avoid an obsticle. Only called if global stopFlag is true.
 *               It puts current reffrence for PID as oldRef. Needed to remember once getting back on original trajectory. 
 *               Rotates left by a defined amount of degrees. Since this is done in a while loop, calling to compute coordinates within the loop. 
 *               Checks weather moving up or down an axis (PID reference is inversely related).
 *               Sets and error in relation to the new reference. 
 *               Once the turn is complete and new reference for PID is set, proceed driving forward. 
 *               Set resumeFlag true, which is tested for maneuvering back on orignal trajectory. PID takes care of the rest.
 * Parameters:   N/A
 * Return:       bool resumeFlag.
 */
void avoidObstacle()
  {
    oldRef = refPos;
    rotateLeft();
    Serial.println("%rotateleft");
    
    float turnReference =xyomega[2]+0.25*PI; // change to define rotation angle
      while (xyomega[2] < turnReference)
      {
             nr = countM1;
             nl = countM2;
             xyPosition(R, D, Nl, Nr, nl, nr, x, y, omega0);
             x = x1;
             y = y1;
             omega0 = omega1;    
      }

    if (axisUp == true)
    {
      refPos=refPos-box;
      error=error-box;
    }
    else
    {
      refPos=refPos+box;
      error=error+box;
    }
    driveForward();
    resumeFlag = true;
  }

/*
 * Summary:      Function to stop and wait if obsticle in front. Calculates time while waiting. 
 *               Continue without avoidance if obsticle dissapears within defined time interval. 
 * Parameters:   N/A
 * Return:       N/A.
 */
void obsticleCheck()
{
   stopFlag = sensor_read();
   stopped=millis();
    while (stopFlag)
    {
      stopMotors();
      stopFlag = sensor_read();
      standing = millis()-stopped;
      
      if (stopFlag == false)
      {        
        driveForward();
      }
      else if (stopFlag == true && standing >= waitingTime && xyomega[1] > 50)
      {
      avoidObstacle();
      stopFlag = false;
      
      }
        previousTime=millis()-1;
    }
}



/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::TRAJECTORY PLANT:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/
void executeTrajectory()
{
    //check if you are at start postiton. drive forward. (do stage 0)
    if (checkPoint0 == true)
    {
      refPos = PIDvector[0]; // sets reference position for PID controller 
      CP=0; // pointer to x coordinate, since we are driving up the obsticale course and we want to stay straight 
      driveForward();
      checkPoint0 = false; // in the next loops, this function will be ignored
      checkPoint1 = true; // getting ready to start next stage
      axisUp = true; // moving up the axis (from y = 30 to y = posVector[1])
    }

    // check if motors travelled forward the required distance (if true, stage 0 finished). Rotate left 90 degrees (start stage 1)
    if (xyomega[1] >= posVector[1] && checkPoint1 == true)
    {
      refPos = PIDvector[0]; 
      CP=0;
      stopMotors(); //stoping motors before switching their states.
      rotateLeft();
      checkPoint1 = false;
      checkPoint2 = true;
    }

    // check if motors rotated (stage 1 finished). drive forward (start stage 2)
    if (xyomega[2] >= posVector[2] && checkPoint2 == true)
    { 
      refPos = PIDvector[1];
      CP=1; //pointer to y coordinate 
      stopMotors();
      driveForward();
      checkPoint2 = false;
      checkPoint3 = true;

    }
    
    // rotate left 90 degrees (stage 3) 
    if (xyomega[0] <= posVector[3] && checkPoint3 == true)
    {      
      refPos = PIDvector[1];
      CP=1;
      stopMotors();
      rotateLeft();
      checkPoint3 = false;
      checkPoint4 = true;
    }

    //drive forward (stage 4)
    if (xyomega[2] >= posVector[4] && checkPoint4 == true)
    {
      refPos = PIDvector[2];
      CP=0;
      stopMotors();
      driveForward();
      checkPoint4 = false;
      checkPoint5 = true;
      axisUp = false;
    }

    // stop, wait for package loading and rotate right 180 degrees (stage 5)
    if (xyomega[1] <= posVector[5] && checkPoint5 == true)
    { 
      refPos = PIDvector[2];
      CP=0;
      stopMotors();
      rotateLeft();
      checkPoint5 = false;
      checkPoint6 = true;
    }
    
    //drive forward (stage 6)
    if (xyomega[2] >= posVector[6] && checkPoint6 == true)
    {
      refPos = PIDvector[2];
      CP=0;
      stopMotors();
      delay(1000); // for loading

      driveForward();
      checkPoint6 = false;
      checkPoint7 = true;
      axisUp = true;
    }

    //turn left 90 degrees (stage 7)
    if (xyomega[1] >= posVector[7] && checkPoint7 == true)
    {
      refPos = PIDvector[2];
      CP=0;
      stopMotors();

      rotateLeft();
      checkPoint7 = false;
      checkPoint8 = true;
    }
    
   // drive forward (stage 8)
    if (xyomega[2] >= posVector[8] && checkPoint8 == true)
    { 
      refPos = PIDvector[3];
      CP=1;
      stopMotors();
      driveForward();
      checkPoint8 = false;
      checkPoint9 = true;
    }

    // rotate left 90 degrees (stage 9)
    if (xyomega[0] <= posVector[9] && checkPoint9 == true)
    {
      refPos = PIDvector[3];
      CP=1;
      stopMotors();
      rotateLeft();
      checkPoint9 = false;
      checkPoint10 = true;
    }

    // drive forward (stage 10)
    if (xyomega[2] >= posVector[10] && checkPoint10 == true)
    {
      refPos = PIDvector[4];
      CP=0;
      stopMotors();
      driveForward();
      checkPoint10 = false;
      checkPoint11 = true;
      axisUp = false;
    }
    
    // finish!! (stage 11)
    if (xyomega[1] <= posVector[11] && checkPoint11 == true)
    {
      stopMotors();
      checkPoint11 = true;
      // wait for reset or something
      int STOP = 1;
      while (int STOP = 1){
        }
    }
}

/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::SETUP:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/

void setup() 
{  
// set digital pinmodes for motors
  pinMode(bridge1APin, OUTPUT); 
  pinMode(bridge2APin, OUTPUT);
  pinMode(bridge3APin, OUTPUT); 
  pinMode(bridge4APin, OUTPUT);

//initialize encoder pins for motors
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT); 
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT); 
  
//Enable pull-up resistors
  digitalWrite(encoder1A, HIGH); 
  digitalWrite(encoder1B, HIGH); 
  digitalWrite(encoder2A, HIGH); 
  digitalWrite(encoder2B, HIGH); 
   
//initializing external interrupt for encoder on rising edge on Phase A on both motors
  attachInterrupt(digitalPinToInterrupt(encoder1A), counterM1, RISING);  
  attachInterrupt(digitalPinToInterrupt(encoder2A), counterM2, RISING);

//modes for US sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
}

/* 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  :::MAIN:::
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
*/
void loop()
{   
   count++; // counts up each loop
   if (count == 2) // change for longer interval between computing coordinates and PID
   {
       nr = countM1; // get the current encoder ticks from the ISR
       nl = countM2;
       xyPosition(R, D, Nl, Nr, nl, nr, x, y, omega0); // calculate new coordinates
       x = x1; // store new coordinates as current coordinates
       y = y1;
       omega0 = omega1;
       computePID(refPos, realPos); // compute PID output
       count = 0;
    }

   // set the refernce position for PID controler back to normal trajectory after avoidance maneuver 
    if (resumeFlag == true && error < 1 && error > -1) 
  {
      refPos = oldRef;
      resumeFlag = false;
  }

    // to prevent obsticle detection while turning, so it would not resume in a straight line without making a full turn
    if (countM1 > 0 && countM2 > 0)
    {
      obsticleCheck();
    }

    executeTrajectory();
}


