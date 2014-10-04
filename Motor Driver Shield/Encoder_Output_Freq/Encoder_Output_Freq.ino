/*

  Left motor = M2, Right motor = M1
  
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5

*/

// Initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
unsigned long prevMicroSec = 0;
unsigned long interval = 1000000; // time unit = us
int logic, m1Count=0, m2Count=0, m1PrevState, m1State, m2PrevState, m2State;

void setup(){  
  // Starting Connections
  Serial.begin(9600);
  
  // Setting wheels to move robot forward
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, HIGH);
  
  analogWrite(m1PWM, 1*255);
  analogWrite(m2PWM, 1*255);
  Serial.println("Setup Complete");
  m2PrevState = digitalRead(3);
  m1PrevState = digitalRead(5);
}

void loop(){
  unsigned long currentMicroSec = micros();
  
  
  m2State = digitalRead(3);
  m1State = digitalRead(5);
  
  if(m2PrevState<m2State){ m2Count++;}
  m2PrevState = m2State;
  
  if(m1PrevState<m1State){ m1Count++;}
  m1PrevState = m1State;
  
  if((currentMicroSec - prevMicroSec) > interval){
    Serial.print("Motor(L) Freq = ");
    Serial.print(m2Count);
    Serial.print(" Motor(R) Freq = ");
    Serial.print(m1Count);
    Serial.print(" Difference in freq = ");
    Serial.print(m2Count-m1Count);
    Serial.println("");
    
    m2Count = 0;
    m1Count = 0;
    prevMicroSec = currentMicroSec;
  }
}


