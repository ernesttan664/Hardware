/*

  Left motor = M2, Right motor = M1
  
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
  
  Values used at home for speed = 600:
  left motor: m2_P = 1.29, m2_I = 0.001, m2_D = 0.1;
  right motor: m1_P = 2.3, m1_I = 0.001, m1_D = 0.1;
  
  Values used at home for speed = 200:
  left motor: m2_P = 1.24, m2_I = 0.001, m2_D = 0.1;
  right motor: m1_P = 1.3, m1_I = 0.001, m1_D = 0.1;
  
  Values used at lounge for speed = 200:
  left motor: m2_P = 1.25, m2_I = 0.001, m2_D = 0.1;
  right motor: m1_P = 1.3, m1_I = 0.001, m1_D = 0.1;
  
  Values used at arena for speed = 200:
  left motor: m2_P = 1.26, m2_I = 0.001, m2_D = 0.1;
  right motor: m1_P = 1.3, m1_I = 0.001, m1_D = 0.1;

*/

#include <PID_v1.h>

// Initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int URPWM = 11; // PWM Output 0-25000US,Every 50US represent 1cm
int URCOMP= 13; // PWM trigger pin
static int sonarDist=0; 
unsigned long prev_ms = 0, prevTime = 0, gridTime = 11000, interval = 400; // 400 time unit = ms
int m1PrevState, m1State, m2PrevState, m2State, command = 1;
double m2DC = 0.2, m1DC = 0.2, m1SpeedAdjustment, m2SpeedAdjustment, m1Count=0.0, m2Count=0.0, targetSpeed = 200;
double m2_P = 1.23, m2_I = 0.001, m2_D = 0.1;
double m1_P = 1.3, m1_I = 0.001, m1_D = 0.1;

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
  
  Serial.println("Setup Complete");
  
}

void loop(){
//  sonarReading();
//  if(sonarDist>20){
//    moveForward();
//  }else{
//    analogWrite(m1PWM, 0);
//    analogWrite(m2PWM, 0);
//  }
  unsigned long currentTime = millis();
  if((currentTime - prevTime)<=(1*gridTime))
    moveForward();
  else{
    prevTime = currentTime;
    analogWrite(m1PWM, 0);
    analogWrite(m2PWM, 0);
    delay(2000);
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
    m2DC = (m2Count+m2SpeedAdjustment)/(800);
    m1DC = (m1Count+m1SpeedAdjustment)/(820);
 
    
    // Print results
//    Serial.print("Motor(L) Freq = ");
//    Serial.print(m2Count);
//    Serial.print(" Motor(R) Freq = ");
//    Serial.print(m1Count);
//    Serial.print(" Difference in freq = ");
//    Serial.print(m2Count - m1Count);
//    Serial.print(" Motor(L) DC Adjustment = ");
//    Serial.print(m2DC);
//    Serial.print(" Motor(R) DC Adjustment = ");
//    Serial.print(m1DC);
//    Serial.println("");
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

void sonarReading(){
  unsigned long pulseDuration = (pulseIn(URPWM, LOW, 10000)) ;
  if(pulseDuration<=50000 && pulseDuration>0)
    sonarDist = pulseDuration/50; // 1cm for every 50us low level stands for 
}

void serialEvent(){
  command = Serial.readString().toInt();
}
