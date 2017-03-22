//Dillon MacEwan student ID 211285035
//sumobotcode
//written for SEM433 Mechatronic Design, Deakin University 2014
//Newping library and code written by Tim Eckel code.google.com/p/arduino-new-ping/
//Pin Change interrupt library source: code.google.com/p/arduino-pinchangeint/
#include <NewPing.h>
#include <PinChangeInt.h>



#define SONAR_NUM     3 // Number or sensors.
#define MAX_DISTANCE 100 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(38, 39, MAX_DISTANCE),
  NewPing(40, 41, MAX_DISTANCE),
  NewPing(42, 43, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
//  NewPing(44, 45, MAX_DISTANCE),
//  NewPing(46, 47 , MAX_DISTANCE),
};

//set pushbutton pins
const int Start = 53; 
const int Mode1 = 49;
const int Mode2 = 51;
//set LED pins
const int Red = 50;
const int Blue = 48;
const int Yellow = 52;
//set motor pins 
const int Dir1 = 30;
const int Dir2 = 31;
const int M1Speed = 8;
const int M2Speed = 9;
//set line detector pins
const int FrontL = 24;
const int FrontR = 25;
const int RearL = 26;
const int RearR = 27;
//Set Ultrasonic Pins
//const int FrontTrig = 38;
//const int FrontEcho = 39;
//const int FLeftTrig = 40;
//const int FLeftEcho = 41;
//const int SLeftTrig = 42;
//const int SLeftEcho = 43;
//const int FRightTrig = 44;
//const int FRightEcho = 45;
//const int SRightTrig = 46;
//const int SRightEcho = 47;

const int Forward = HIGH;
const int Back = LOW;
const int Left = HIGH;
const int Right = LOW;
const int LeftUS = 0;
const int CentreUS = 1;
const int RightUS = 2;

long taskTime = 1000;
int search = 0;
long starting = millis();



int mode = 0;
int start =  LOW;



void setup()
{
  //initialise all pins
  //initialise LED pins
  pinMode(Red, OUTPUT);
  pinMode(Blue, OUTPUT);
  pinMode(Yellow, OUTPUT);
  //intitialise button pins
  pinMode(Start, INPUT);
  pinMode(Mode1, INPUT);
  pinMode(Mode2, INPUT);
  //initialise motor pins
  pinMode(Dir1, OUTPUT);
  pinMode(Dir2, OUTPUT);
  pinMode(M1Speed, OUTPUT);
  pinMode(M2Speed, OUTPUT);
  //initialise line sensor pins
  pinMode(FrontL, INPUT);
  pinMode(FrontR, INPUT);
  pinMode(RearL, INPUT);
  pinMode(RearR, INPUT);
  
  digitalWrite (Yellow, LOW);
  digitalWrite (Red, LOW);
  digitalWrite (Blue, LOW);
  
 search = 0;
  
  
  
  if (digitalRead(Mode1) == HIGH)
    mode = 1;
  else if (digitalRead(Mode2) == HIGH)
    mode = 2;
  else
    mode = 0;
    
    attachInterrupt(FrontL, &edgeFind, FALLING);
    attachInterrupt(FrontR, &edgeFind, FALLING);
    attachInterrupt(RearL, &edgeFind, FALLING);
    attachInterrupt(RearR, &edgeFind, FALLING);
    
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
    while (start == LOW)//wait for start button to be pushed
  {
    int pushStart = digitalRead (Start);
    if (pushStart == 1)
    start = HIGH;
    delay(50);
    
  }
  
  long starting = (millis() + 5000);
  
  delay (5000);

}

void loop()
{
  
  
  
  if (mode == 1)
    mode1();
    else if (mode == 2)
      mode2();
    else if (mode == 0)
      mode0();
  
int pushStart = digitalRead (Start);
    if (pushStart == 1)
    {
    delay(50);
    softReset();
    }
  if ((millis() - starting) > taskTime)
    softReset;
   

}

//mode functions
void mode0() //compete mode
  {
    taskTime = 200000;
    digitalWrite (Blue, HIGH);
    runTime (255, HIGH, 300);
    spinTime (100,Right,300);
    for (uint8_t i = 0; i < SONAR_NUM; i++) 
    { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) 
    {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
    }
     search++;
    if (search == 2)
      softReset;
    
  }

void mode1() //attack mode
  {
    taskTime = 150000;
    digitalWrite (Red, HIGH);
    runTime (255, Forward, 300);
    spinTime (100,Right,300);
    for (uint8_t i = 0; i < SONAR_NUM; i++) 
    { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) 
    {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
    }
  
  }

void mode2() //evade mode
  {
    taskTime = 70000;
    digitalWrite (Yellow, HIGH);
    runTime (255, Forward, 300);
    spinTime (255,Right,50);
    while (digitalRead(RearL)==HIGH && digitalRead(RearR)==HIGH)
    {
      run (255, Back);
    }
    brake();
    for (uint8_t i = 0; i < SONAR_NUM; i++) 
    { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) 
    {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
    }
  
  }

//motor control functions

void brake() //Stops both motors
{
analogWrite(M1Speed, 0);
analogWrite(M2Speed, 0);
}

void spin (int Vel, int Dir) //spins on the spot
{
  int OpDir;
  if (Dir == HIGH)
    OpDir = LOW;
    else
    OpDir = HIGH;
  analogWrite(M1Speed, Vel);
  analogWrite(M2Speed, Vel);
  digitalWrite(Dir1, Dir);
  digitalWrite(Dir2, OpDir);
}

void run (int Vel, int Dir) //runs in a straight line, forward or reverse
{
  analogWrite(M1Speed, Vel);
  analogWrite(M2Speed, Vel);
  digitalWrite(Dir1, Dir);
  digitalWrite(Dir2, Dir);
}

//steers left or right Tdir 0 = left 1 = right
void turn (int Vel, int TSpeed, int Tdir, int Dir)

{
  int TSpeed1;
  int TSpeed2;
  if (Tdir = 0)
    {
      TSpeed1 = TSpeed;
      TSpeed2 = 1;
    }
    else
      {
        TSpeed1 = 1;
        TSpeed2 = TSpeed;
      }
  analogWrite(M1Speed, Vel/TSpeed1);
  analogWrite(M2Speed, Vel/TSpeed2);
  digitalWrite(Dir1, Dir);
  digitalWrite(Dir2, Dir);
}

void spinTime (int Vel, int Dir, long Time) //spins on the spot
{
  int OpDir;
  if (Dir == HIGH)
    OpDir = LOW;
    else
    OpDir = HIGH;
  long startTime = millis();
  long currentTime = millis();
  while (currentTime - startTime < Time)
   { 
    analogWrite(M1Speed, Vel);
    analogWrite(M2Speed, Vel);
    digitalWrite(Dir1, Dir);
    digitalWrite(Dir2, OpDir);
    currentTime = millis();
   }
   brake();
}

void runTime (int Vel, int Dir, long Time) //runs in a straight line, forward or reverse
{
  long startTime = millis();
  long currentTime = millis();
  while (currentTime - startTime < Time)
   { 
    analogWrite(M1Speed, Vel);
    analogWrite(M2Speed, Vel);
    digitalWrite(Dir1, Dir);
    digitalWrite(Dir2, Dir);
    currentTime = millis();
   }
   brake();
}

//steers left or right Tdir 0 = left 1 = right
void turnTime (int Vel, int TSpeedN, int TSpeedD, int Tdir, int Dir, long Time)

{
  int TSpeed1N;
  int TSpeed2N;
  int TSpeed1D;
  int TSpeed2D;
  if (Tdir = 0)
    {
      TSpeed1N = TSpeedN;
      TSpeed2N = 1;
      TSpeed1D = TSpeedD;
      TSpeed2D = 1;
    }
    else
      {
        TSpeed1N = 1;
        TSpeed2N = TSpeedN;
        TSpeed1D = 1;
        TSpeed2D = TSpeedD;
      }
  long startTime = millis();
  long currentTime = millis();
  while (currentTime - startTime < Time)
   { 
      analogWrite(M1Speed, (Vel/TSpeed1D)*TSpeed1N);
      analogWrite(M2Speed, (Vel/TSpeed2D)*TSpeed2N);
      digitalWrite(Dir1, Dir);
      digitalWrite(Dir2, Dir);
      currentTime = millis();
   }
   brake();
}

void echoCheck() 

{ 
  if (sonar[currentSensor].check_timer())
  {
    if (mode == 0 || mode ==1)
    {
      if (currentSensor == CentreUS)
      {
        while (digitalRead(FrontL)==HIGH && digitalRead(FrontR)==HIGH)
        {
        run (255,Forward);
        }
        brake();
        turnTime (255,1,2,Right,Back,200);
        turnTime (255,1,2,Left,Back,200);
        spinTime (100,Right,300);
    for (uint8_t i = 0; i < SONAR_NUM; i++) 
    { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) 
    {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      search++;
    }
      }        
    }
    if (mode == 2)
    {
      if (currentSensor == CentreUS || currentSensor == LeftUS)
      {
        turnTime (255,1,2,Right,Forward,200);
        turnTime (255,1,2,Left,Forward,200);
      }
      else if (currentSensor == RightUS)
      {
        turnTime (255,1,2,Left,Forward,200);
        turnTime (255,1,2,Right,Forward,200);
      }
    }
  }
  }
} 


void edgeFind()
{
  
  
  {
    if (mode != 0)
    {
      if (FrontL == LOW)
        {
          turnTime (255,1,2,Right,Back,200);
          turnTime (255,1,2,Left,Back,200);
        }
      else if (FrontR == LOW)
        {
          turnTime (255,1,2,Left,Back,200);
          turnTime (255,1,2,Right,Back,200);
        }  
     }
    if (mode != 2)
    {
      if (RearL == LOW)
        {
          turnTime (255,1,2,Right,Forward,200);
          turnTime (255,1,2,Left,Forward,200);
        }
      else if (RearR == LOW)
        {
          turnTime (255,1,2,Left,Forward,200);
          turnTime (255,1,2,Right,Forward,200);
        }  
     }
  }
}

void softReset()
{
  delay(100);
  asm volatile ("  jmp 0");
}
