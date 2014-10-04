/*
  Left motor = M2, Right motor = M1

  Connections             Arduino Pin
  M2INA output --->       pin 7
  M1INA output --->       pin 2
  Left IR sensor          A2
  Right IR sensor         A3

*/
#include "RunningAverage.h"
#define totalSamples 2000
#define threshold 10

int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int samplesRemaining = totalSamples;
int leftSensor, rightSensor, distDifference;

RunningAverage dist(totalSamples);
RunningAverage b(totalSamples);
void setup(){
  Serial.begin(9600);
  dist.clear();
  b.clear();
}

void loop(){
  computeMovingAverage();
  printAvg();
  repositionRobot();
  
}

void repositionRobot(){
  
  // setting wheels to move robot backwards
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, HIGH);
  
  while(true){ 
    // if difference > threshold, robot is facing more towards left, need to rotate right wheel backwards 
    if(distDifference > threshold){
      Serial.println("Reverse right wheel");
      analogWrite(m2PWM, 0.5*255);
      delay(50);
      analogWrite(m2PWM, 0*255);
    } else if(distDifference < -threshold){
      // if difference < 0, robot is facing more towards right, need to rotate left wheel backwards
      Serial.println("Reverse left wheel");
      analogWrite(m1PWM, 0.5*255);
      delay(50);
      analogWrite(m1PWM, 0*255);
    }
    else{break;}
    computeMovingAverage();
  }
  
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

void printAvg(){
  Serial.print("Left sensor = ");
  Serial.print(leftSensor);
  Serial.print(" Right sensor =");
  Serial.print(rightSensor);
  Serial.print(" Difference = ");
  Serial.println(leftSensor-rightSensor); 
}
