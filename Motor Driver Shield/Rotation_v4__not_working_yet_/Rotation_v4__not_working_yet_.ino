/*
  Left motor = M2, Right motor = M1
  
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5

*/

#include <PinChangeInt.h>
#define pi 3.14159265359
#define left_rotation_interval 1100
#define left_rotation_pi_const 0.78
#define rotation_sampling_interval 100

// Initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int count=0;
double left_rotation_dc[left_rotation_interval/rotation_sampling_interval] ;
void setup(){
  
  Serial.begin(115200);
  PCintPort::attachInterrupt(PIN3, &interrrupt_count, RISING);
  
  // Compute left rotation dc
  for(int i=0,t=0; i<(left_rotation_interval/rotation_sampling_interval); i++,t+=rotation_sampling_interval){
    left_rotation_dc[i] = abs(0.51*sin(left_rotation_pi_const*pi*t*0.001)*sin(left_rotation_pi_const*pi*t*0.001));
  }
  
  
  
  Serial.println("Connection Established");
}

void loop(){
  //rotateLeft90();
  //delay(1500)
  rotateRight90();
  delay(1500);
}

void rotateLeft90(){
  /// setting wheel direction to rotate robot left
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(300);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(count<440){
    delay(1);
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  count=0;
}

void rotateRight90(){
  /// setting wheel direction to rotate robot left
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, LOW);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(300);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(count<410){
    delay(1);
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  count=0;
}

void interrrupt_count(){
  count++;
}
