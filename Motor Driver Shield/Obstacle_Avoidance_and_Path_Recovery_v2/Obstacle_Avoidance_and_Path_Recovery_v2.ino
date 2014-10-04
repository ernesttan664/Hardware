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
  
  Right Rotation Time at home: 220
  Left Rotation Time at home: 230
  Right Rotation TIme at lounge: 330
  Left Rotation TIme at lounge: 340

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
static int sonarDist=0, obstacleDetected=0;
unsigned long prev_ms = 0, interval = 100; // time unit = ms
int m1PrevState, m1State, m2PrevState, m2State, command = 1, rightRotateTime = 700, leftRotateTime = 700, forwardTime = 800;
double m2DC = 0.2, m1DC = 0.2, m1SpeedAdjustment, m2SpeedAdjustment, m1Count=0.0, m2Count=0.0, targetSpeed = 200;
double m2_P = 1.1, m2_I = 0.0001, m2_D = 0.001;
double m1_P = 1.18, m1_I = 0.0001, m1_D = 0.001;

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
  // if no obstruction
    //move forward
  // else rotate 90 deg right
  // do move 2 grids forward while no obstruction detected on left side sensor
  // rotate 90 left
  // move forward until obstruction detected on left side sensor
  // stop robot
  // move robot 2 grids forward
  // rotate 90 deg left
  // move forward 2 grids
  // rotate 90 deg right
  // move forward
  sonarReading();
  while(sonarDist>15||sonarDist==0){
    moveForward();
    sonarReading();
  }
  rotateRight90();
  do{
    moveForward();
    delay(1*forwardTime);
    stopRobot();
    delay(1000);
    obstacleDetected++;
  }while(leftSideSensorReading()<=20);
  
  moveForward();
  delay(2*forwardTime);
  stopRobot();
  rotateLeft90();
  
  while(leftSideSensorReading()>=21){
    moveForward();
    delay(forwardTime);
  }
  stopRobot();
  delay(1000);
  moveForward();
  delay(3*forwardTime);
  stopRobot();
  delay(1000);
  rotateLeft90();
  
  moveForward();
  delay(3*obstacleDetected*forwardTime);
  stopRobot();
  
  rotateRight90();
  
  moveForward();
  delay(2000);
  stopRobot();
  delay(10000);
  
}

void rotateRight90(){
  // setting wheel direction to rotate robot clockwise
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, LOW);
  
  analogWrite(m2PWM, 0.5*255);
  analogWrite(m1PWM, 0.5*255);
  
  delay(rightRotateTime);
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, LOW);
  delay(500);
}
void rotateLeft90(){
  // setting wheel direction to rotate robot clockwise
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM, 0.5*255);
  analogWrite(m1PWM, 0.5*255);
  
  delay(leftRotateTime);
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, LOW);
  delay(500);
}

void stopRobot(){
  analogWrite(m1PWM, 0*255);
  analogWrite(m2PWM, 0*255);
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

int leftSideSensorReading(){
 return ((7100/(analogRead(4)-3))-3); 
}

int leftDiagSensorReading(){
 return ((6500/(analogRead(2)-4))-3); 
}

int rightDiagSensorReading(){
 return ((5210/(analogRead(3)-10))-3); 
}

int rightSideSensorReading(){
 return ((4800/(analogRead(5)-35))-3); 
}

void sonarReading(){
  unsigned long pulseDuration = (pulseIn(URPWM, LOW, 10000)) ;
  if(pulseDuration<=50000 && pulseDuration>0)
    sonarDist = pulseDuration/50; // 1cm for every 50us low level stands for 
}

void serialEvent(){
  command = Serial.readString().toInt();
}
