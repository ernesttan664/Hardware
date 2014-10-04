/*

  Left motor = M2, Right motor = M1
  
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5
  
  Values used at home:
  left motor: m2_P = 2.0, m2_I = 0.001, m2_D = 0.1;
  right motor: m1_P = 2.0, m1_I = 0.001, m1_D = 0.1;
  
*/

#include <PID_v1.h>
#include "RunningAverage.h"
#define totalSamples 2000
#define threshold 10

// Initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int leftSensor, rightSensor, distDifference;
unsigned long prevMicroSec = 0;
unsigned long interval = 500; // time unit = ms
int logic, m1PrevState, m1State, m2PrevState, m2State, command = 1;
double m2DC = 0.2, m1DC = 0.2, m1SpeedAdjustment, m2SpeedAdjustment, m1Count=0.0, m2Count=0.0, targetSpeed = 600.0;
double m2_P = 2.0, m2_I = 0.001, m2_D = 0.1;
double m1_P = 2.0, m1_I = 0.001, m1_D = 0.1;

//Specify the links and initial tuning parameters
PID leftPID(&m2Count, &m2SpeedAdjustment, &targetSpeed, m2_P, m2_I , m2_D, DIRECT);
PID rightPID(&m1Count, &m1SpeedAdjustment, &targetSpeed, m1_P, m1_I, m1_D, DIRECT);

RunningAverage dist(totalSamples);

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
  
  Serial.println("Setup Complete");
  
}

void loop(){
  switch(command){
    case 1:
      if(analogRead(2) <300){
        moveForward();
      }else{
        analogWrite(m1PWM, 0);
        analogWrite(m2PWM, 0);
        repositionRobot();
        command = 2;      
      }
      break;
    case 2:
        Serial.println(command);
        analogWrite(m1PWM, 0);
        analogWrite(m2PWM, 0);
      break;
  }
  
  
}

void moveForward(){
  // Setting wheels to move
  analogWrite(m1PWM, m1DC*255);
  analogWrite(m2PWM, m2DC*255);
  
  unsigned long currentMicroSec = millis();
  
  // Compute FSM to track rate of rising edges per 0.5 sec from encoder output
  computeFSM();
  
  
  if((currentMicroSec - prevMicroSec) > interval){
    
    leftPID.Compute();
    rightPID.Compute();
    m2DC = (m2Count+m2SpeedAdjustment)/1150;
    m1DC = (m1Count+m1SpeedAdjustment)/1150;
 
    
    // Print results
    Serial.print("Motor(L) Freq = ");
    Serial.print(m2Count);
    Serial.print(" Motor(R) Freq = ");
    Serial.print(m1Count);
    Serial.print(" Difference in freq = ");
    Serial.print(m2Count - m1Count);
    Serial.print(" Motor(L) Adjustment = ");
    Serial.print(m1SpeedAdjustment);
    Serial.print(" Motor(R) Adjustment = ");
    Serial.print(m2SpeedAdjustment);
    Serial.println("");
    
    m2Count = 0;
    m1Count = 0;
    prevMicroSec = currentMicroSec;
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

void repositionRobot(){
  
  // setting wheels to move robot backwards
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, HIGH);
  
  while(true){
    computeMovingAverage(); 
    // if difference > threshold, robot is facing more towards left, need to rotate right wheel backwards 
    if(distDifference > threshold){
      Serial.println("Reverse right wheel");
      analogWrite(m2PWM, 0.2*255);
      analogWrite(m1PWM, 0*255);
    } else if(distDifference < -threshold){
      // if difference < 0, robot is facing more towards right, need to rotate left wheel backwards
      Serial.println("Reverse left wheel");
      analogWrite(m1PWM, 0.2*255);
      analogWrite(m2PWM, 0*255);
    }
    else{
      analogWrite(m2PWM, 0*255);
      analogWrite(m1PWM, 0*255);
      break;
    }
  }
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, LOW);
 
}

void computeMovingAverage(){
  // Compute MA for left sensor
  for(int i=totalSamples ;i>0 ;i--)
    dist.addValue(analogRead(2));
  leftSensor = dist.getAverage();
  dist.clear();
  
  // Compute MA for right sensor
  for(int i=totalSamples ;i>0 ;i--)
    dist.addValue(analogRead(3));
  rightSensor = dist.getAverage();
  dist.clear();
  distDifference = leftSensor - rightSensor;
}

void serialEvent(){
  command = Serial.readString().toInt();
}
