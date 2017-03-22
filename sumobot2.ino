//Dillon MacEwan student ID 211285035
//sumobotcode
//written for SEM433 Mechatronic Design, Deakin University 2014
//Newping library and code written by Tim Eckel code.google.com/p/arduino-new-ping/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <NewPing.h>

//newPing setup

#define SONAR_NUM     3 // Number or sensors.
#define MAX_DISTANCE 60 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 35 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(38, 39, MAX_DISTANCE),
  NewPing(40, 41, MAX_DISTANCE),
  NewPing(42, 43, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
//  NewPing(44, 45, MAX_DISTANCE),
//  NewPing(46, 47 , MAX_DISTANCE), //spares defined
};



//set pushbutton pins
const int Start = 53; 
const int Mode1 = 51;
const int Mode2 = 49;
//set LED pins
const int Red = 50;
const int Blue = 48;
const int Yellow = 52;
//set motor pins 
const int Dir1 = 30;
const int Dir2 = 31;
const int M1Speed = 8;
const int M2Speed = 9;
//set direction control
const int Forward = HIGH;
const int Back = LOW;
const int Left = HIGH;
const int Right = LOW;
//define ultrasonc labels
const int LeftUS = 0;
const int CentreUS = 1;
const int RightUS = 2;
const int Range = 40;
volatile int Object = 0;
//preset mode and start
int mode = 0;
int start =  0;

void setup()
{

  //initialise all pins
  //initialise LED pins
  pinMode(Red, OUTPUT);
  pinMode(Blue, OUTPUT);
  pinMode(Yellow, OUTPUT);
  //intitialise button pins
  pinMode(Start,  INPUT_PULLUP);
  pinMode(Mode1,  INPUT_PULLUP);
  pinMode(Mode2,  INPUT_PULLUP);
   //initialise motor pins
  pinMode(Dir1, OUTPUT);
  pinMode(Dir2, OUTPUT);
  pinMode(M1Speed, OUTPUT);
  pinMode(M2Speed, OUTPUT);
  //initialise line sensors
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  pinMode(20,INPUT);
  pinMode(21,INPUT);
  
  
 
  
  
  brake();
  
  delay (1000);
  digitalWrite (Yellow, HIGH);
  delay (300);
  digitalWrite (Yellow, LOW);
  delay (200);
  
 //newPing code to set up ping timing
   pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
  
 
    
  
        while (start == 0)//wait for start button to be pushed
  {
    int pushStart = digitalRead (Start);
    if (pushStart == LOW)
    start = 1;
    delay(50);
    
  
  }
  
  // set mode according to input
    if (digitalRead(Mode1) == LOW)
    mode = 1;
  else if (digitalRead(Mode2) == LOW)
    mode = 2;
  else
    mode = 0;
    
  digitalWrite (Red, HIGH); //signal pushbutton
  delay (300);
  digitalWrite (Red, LOW);
  
  delay (4500); //delay 5s before doing anything
//  sei();

digitalWrite (Blue, HIGH); //signal start
  delay (300);
  digitalWrite (Blue, LOW);
  
  

//attach interrupts for line sensors
  attachInterrupt(2, FrontL, LOW);
  attachInterrupt(3, FrontR, LOW);
  attachInterrupt(4, RearL, LOW);
  attachInterrupt(5, RearR, LOW);
//  
  
spin(200,Right);
delay(1100);
if (mode == 2)
  run(200,Back);
}

void loop()
{

  
//  
  if (mode == 1)
    mode1();
    else if (mode == 2)
      mode2();
    else if (mode == 0)
      mode0();
  
int pushStart = digitalRead (Start); //reset if start/reset button is pushe
if (digitalRead (Start)== LOW)
    {
      digitalWrite (Blue, LOW);
      digitalWrite (Red, LOW);
      digitalWrite (Yellow, LOW);
    delay(50);
    softReset();
    }
 
   

}

void mode0() //compete mode
  
  {
   
    run(200,Forward);
    spin (80,Left);
    for (uint8_t i = 0; i < 5; i++) 
  {    
    scan();
  }
        while (Object == 1)
        {
          scan();
        }
        turnTime (200,1,2,Left,Forward,400);
        runTime (200,Forward,500);
        turnTime (200,1,2,Right,Forward,800);
        spin (80,Right);
        for (uint8_t i = 0; i < 5; i++) 
  {    
    scan();
  }
        while (Object == 1)
        {
          scan();
        }
        turnTime (200,1,2,Right,Forward,400);
        runTime (200,Forward,500);
        turnTime (200,1,2,Left,Forward,800);
  }
  
  void mode1() //attack mode
  
  {
    spin (80,Left);
    for (uint8_t i = 0; i < 5; i++) 
  {    
    scan();
  }
        while (Object == 1)
        {
          scan();
        }
        turnTime (200,1,2,Left,Forward,400);
        runTime (200,Forward,500);
        turnTime (200,1,2,Right,Forward,800);
        spin (100,Right);
        for (uint8_t i = 0; i < 5; i++) 
  {    
    scan();
  }
        while (Object == 1)
        {
          scan();
        }
        turnTime (200,1,2,Right,Forward,400);
        runTime (200,Forward,500);
        turnTime (200,1,2,Left,Forward,800);
  }
  
  void mode2() //evade mode
  {
    digitalWrite (Blue, HIGH);
    scan();
  }
  

//motor control functions

void brake() //Stops both motors
{
analogWrite(M1Speed, 0);
analogWrite(M2Speed, 0);
delay(5);
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

//steers left or right Tdir 0 = left 1 = right, forward or backwrds, at a diff ratio of N/D
void turn (int Vel, int TSpeedN, int TSpeedD, int Tdir, int Dir)

{
  int TSpeed1N;
  int TSpeed2N;
  int TSpeed1D;
  int TSpeed2D;
  if (Tdir == LOW)
    {
      TSpeed1N = TSpeedN;
      TSpeed2N = 1;
      TSpeed1D = TSpeedD;
      TSpeed2D = 1;
    }
    else if (Tdir == HIGH )
      {
        TSpeed1N = 1;
        TSpeed2N = TSpeedN;
        TSpeed1D = 1;
        TSpeed2D = TSpeedD;
      }
  
   
      analogWrite(M1Speed, (Vel/TSpeed1D)*TSpeed1N);
      analogWrite(M2Speed, (Vel/TSpeed2D)*TSpeed2N);
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
  if (Tdir == LOW)
    {
      TSpeed1N = TSpeedN;
      TSpeed2N = 1;
      TSpeed1D = TSpeedD;
      TSpeed2D = 1;
    }
    else if (Tdir == HIGH )
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


//Ultrasonic Routines - modified from the newPing page

void echoCheck() // If ping echo, set distance to array.
{ 
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}



void scan ()//scans for objects
{
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) 
  {
    if (millis() >= pingTimer[i]) 
    {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1)
        oneSensorCycle(); // Call action routine
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
  digitalWrite (Blue, LOW);
}

void oneSensorCycle() //process the data after each scan cycle
{
  if (mode == 2)//evade instructions (mode2)
    {
      if (cm[CentreUS] < Range && cm[CentreUS] > 0)
      {
        digitalWrite (Yellow, HIGH);
        evadeR();
        
      }
      else if (cm[RightUS] < Range && cm[RightUS] > 0)
      {
        digitalWrite (Yellow, HIGH);
        evadeR();
      }
      else if (cm[LeftUS] < Range && cm[LeftUS] > 0)
      {
       digitalWrite (Yellow, HIGH);
        evadeL();
      }
      else
      {
        digitalWrite (Red, LOW);
        digitalWrite (Yellow, LOW);
        digitalWrite (Blue, LOW);
      }
    }
   if (mode == 1)
    {
      if (cm[CentreUS] < Range && cm[CentreUS] > 0)
      {
        digitalWrite (Blue, HIGH);
        run(200,Forward);
        Object = 1;
      }
      else if (cm[RightUS] < Range && cm[RightUS] > 0)
      {
        digitalWrite (Red, HIGH);
        turn(250,1,2,Right,Forward);
        Object = 1;
      }
      else if (cm[LeftUS] < Range && cm[LeftUS] > 0)
      {
        digitalWrite (Red, HIGH); 
        turn(250,1,2,Left,Forward);
        Object = 1;
      }
      else
      {
        digitalWrite (Red, LOW);
        digitalWrite (Yellow, LOW);
        digitalWrite (Blue, LOW);
        Object = 0;
        brake();
      }
    } 
 
  }

//define line spotting interrupt functions
void FrontL()//int2
{
  Object = 0;
  digitalWrite(Red,LOW);
  digitalWrite(Blue,HIGH);
  brake();
  runTime(100,Back,100);
  brake();
  spinTime(200,Right,300);
  brake();
}

void FrontR()//int3
{
  digitalWrite(Red,LOW);  
  digitalWrite(Blue,HIGH);
    

  Object = 0;
  brake();
  runTime(100,Back,100);
  brake();
  spinTime(200,Right,300);
  brake();
  
}


void RearL()//int4
{
  digitalWrite(Red,LOW);
  digitalWrite(Yellow,HIGH);
    Object = 0;
  brake();
  runTime(200,Forward,100);

}

void RearR()//int5
{
  digitalWrite(Red,LOW);
  digitalWrite(Yellow,HIGH);
  Object = 0;
  brake();
  runTime(200,Forward,100);
  
}

//evade functions - replace delays with time functions!!

void evadeR()//evade object from right
{
  
  digitalWrite (Yellow, HIGH);
  spin(200,Left);
  delay(400);
  run(200,Forward);
  delay(600);
  spin(250,Right);
  delay(400);
  brake();
  
}

void evadeL()//evade object from left
{
    
  digitalWrite (Red, HIGH);
    spin(200,Right);
    delay(400);
    run(200,Forward);
    delay(600);
    spin(200,Left);
    delay(400);
    brake();
    
}


  void softReset() //software reset
{
  delay(100);
  asm volatile ("  jmp 0");
}
