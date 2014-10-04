/*

  Left motor = M2, Right motor = M1
  Left motor max freq 1290 (M2)
  Right motor max freq 1230 (m1)
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5
  
  Arduino          Sensor
  Vcc              Pin 1 VCC (URM V3.2)
  GND              Pin 2 GND (URM V3.2)
  Pin 11           Pin 4 PWM (URM V3.2)
  Pin 13           Pin 6 COMP/TRIG (URM V3.2)
  Pin A2           Left Diagonal IR Sensor
  Pin A3           Right Diag IR Sensor
  Pin A4           Left Side IR Sensor
  Pin A5           Right Side IR Sensor
  
  
  
  Values used at home for speed = 250:
  left motor: m2_P = 1.105, m2_I = 0.001, m2_D = 0.01;
  right motor: m1_P = 1.1, m1_I = 0.001, m1_D = 0.01;
  
  Values used at hardware proj lab for speed = 600:
  left motor: m2_P = 1.114, m2_I = 0.001, m2_D = 0.01;
  right motor: m1_P = 1.101, m1_I = 0.001, m1_D = 0.01;
  150cm took approx 6600ms
  
  Values used at software lab 3 for speed = 600:
  left motor: m2_P = 1.1, m2_I = 0.0001, m2_D = 0.001;
  right motor: m1_P = 1.10000007, m1_I = 0.0001, m1_D = 0.001;
  150cm took approx 6600ms
  
  commands needed:
  turn clockwise 45
  turn clockwise 90
  turn anticlockwise 45
  turn anticlockwise 90
  forward
  stop (for emergency use)

*/

#include <PID_v1.h>

// Initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
static int sonarDist=0; 
unsigned long prev_ms = 0, prev_ms_dist;
unsigned long interval = 400 ; // time unit = ms
unsigned long dist_interval; // time unit = ms, 150cm took 660ms at speed=600
int m1PrevState, m1State, m2PrevState, m2State, command = 0;
double m2DC = 0.0, m1DC = 0.0, m1SpeedAdjustment, m2SpeedAdjustment, m1Count=0.0, m2Count=0.0, targetSpeed = 600.0, targetDist = 100.0;
double m2_P = 1.1, m2_I = 0.0001, m2_D = 0.001;
double m1_P = 1.3, m1_I = 0.0001, m1_D = 0.001;

//Specify the links and initial tuning parameters
PID leftPID(&m2Count, &m2SpeedAdjustment, &targetSpeed, m2_P, m2_I , m2_D, DIRECT);
PID rightPID(&m1Count, &m1SpeedAdjustment, &targetSpeed, m1_P, m1_I, m1_D, DIRECT);


void setup(){  
  // Starting Connections
  Serial.begin(9600);
  
  // Setting wheels to move robot forward
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, LOW);
  
  // Setting initial states of FSMs
  m2PrevState = digitalRead(3);
  m1PrevState = digitalRead(5);
  
  //tell the PID to range between -2 and 2
  leftPID.SetOutputLimits(-800,800);
  rightPID.SetOutputLimits(-800,800);
  
  //turn the PID on
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
  dist_interval=targetDist*6600/150.0;
  Serial.println("Setup Complete");
  
}

void loop(){
  unsigned long current_ms = millis();
  if((current_ms - prev_ms_dist)<dist_interval){
    moveForward();
    
  }else{
    analogWrite(m1PWM, 0);
    analogWrite(m2PWM, 0);      
  }
}

void moveForward(){
  // Setting wheels to move
  analogWrite(m1PWM, m1DC*255);
  analogWrite(m2PWM, m2DC*255);
  
  unsigned long current_ms = millis();
  // Compute FSM to track rate of rising edges per 0.5 sec from encoder output
  computeFSM();
  
  if((current_ms - prev_ms) > interval){
    
    leftPID.Compute();
    rightPID.Compute();
    m2DC = (m2Count+m2SpeedAdjustment)/1290;
    m1DC = (m1Count+m1SpeedAdjustment)/1230;
    
    m2Count = 0;
    m1Count = 0;
    prev_ms = current_ms;
  }
}

void computeFSM(){
  m2State = digitalRead(3);
  m1State = digitalRead(5);
  
  if(m2PrevState<m2State){ m2Count++;}
  m2PrevState = m2State;
  
  if(m1PrevState<m1State){ m1Count++;}
  m1PrevState = m1State;
}


