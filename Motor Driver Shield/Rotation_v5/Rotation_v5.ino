/*
  Left motor = M2, Right motor = M1
  
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5

*/

#include <PinChangeInt.h>

// Initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
volatile int m2Count=0, m1Count=0;
void setup(){
  
  Serial.begin(115200);
  PCintPort::attachInterrupt(PIN3, &compute_m2_count, RISING);
  PCintPort::attachInterrupt(PIN5, &compute_m1_count, RISING);
  Serial.println("Connection Established");
}

void loop(){
//  rotateLeft90();
//  delay(1500);
  rotateRight90();
  delay(1500);
}

void rotateLeft90(){
  m2Count=0; m1Count=0;
  int avgCount=0;
  // setting wheel direction to rotate robot left
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(300);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(avgCount<429){
    avgCount = (m2Count+m1Count)/2;
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void rotateRight90(){
  m2Count=0; m1Count=0;
  int avgCount=0;
  // setting wheel direction to rotate robot left
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, LOW);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(300);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(avgCount<415){
    avgCount = (m2Count+m1Count)/2;
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void compute_m2_count(){
  m2Count++;
}

void compute_m1_count(){
  m1Count++;
}
