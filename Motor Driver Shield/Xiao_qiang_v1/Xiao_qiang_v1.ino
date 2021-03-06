/*
  
  Left motor = M2, Right motor = M1
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5
  
  Robot Communications Protocol v1.0
  
  Setup Phase:
    Send "Connection Established" once serial link is available and enter Exploration Phase
  Exploration Phase:
    Whenever a command finishes execution, send an "ACK n" back where n = command
    Sensor data is relayed back at the end of each movement command
  At hardware project lab arena, speed = 600, interval = 100, 1 grid movement = 350
  m2_P = 1.1, m2_I = 0.0001, m2_D = 0.001;
  m1_P = 1.11, m1_I = 0.0001, m1_D = 0.001;
*/
#include <PID_v1.h>
#define samples 21

// initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int URPWM = 11; // PWM Output 0-25000US,Every 50US represent 1cm
int URCOMP= 13; // PWM trigger pin
unsigned long prev_ms = 0, sensor_cooldown_duration = 1000;
unsigned long interval = 100 ; // time unit = ms
unsigned long _1_grid_movement_duration = 380; // time unit = ms, 150cm took 660ms at speed=600
int m1PrevState, m1State, m2PrevState, m2State, command=0, sonarDist=0;
double m2DC = 0.0, m1DC = 0.0, m1SpeedAdjustment, m2SpeedAdjustment, m1Count=0.0, m2Count=0.0, targetSpeed = 600.0;
double m2_P = 1.1, m2_I = 0.0001, m2_D = 0.001;
double m1_P = 1.13, m1_I = 0.0001, m1_D = 0.001;
String commandBuffer = "";
boolean sendSensorReading=true;
int leftSideSensor[samples], leftDiagSensor[samples], rightSideSensor[samples], rightDiagSensor[samples];
int leftSideSensorMedian=0, leftDiagSensorMedian=0, rightDiagSensorMedian=0, rightSideSensorMedian=0;
int obstaclePositions[5];
int leftRotationTime=2300, rightRotationTime=2100, left180RotationTime = 4500;
int leftRotationAdjustmentTime = 50, rightRotationAdjustmentTime = 50;

//Specify the links and initial tuning parameters
PID leftPID(&m2Count, &m2SpeedAdjustment, &targetSpeed, m2_P, m2_I , m2_D, DIRECT);
PID rightPID(&m1Count, &m1SpeedAdjustment, &targetSpeed, m1_P, m1_I, m1_D, DIRECT);


void setup() {
  // setting String buffer size 
  commandBuffer.reserve(200);
  
  // Setting initial states of FSMs
  m2PrevState = digitalRead(3);
  m1PrevState = digitalRead(5);
  
  //tell the PID to range between -2 and 2
  leftPID.SetOutputLimits(-800,800);
  rightPID.SetOutputLimits(-800,800);
  
  //turn the PID on
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
  Serial.begin(9600); // set the baud rate
  
  // Establishing Communications as per RCPv1.0
  while(!Serial);  
  Serial.println("Connection Established"); // print "Connection Established" once
}

void loop() {
  switch(command){
    case 0:  // idle state, send readings once and check if there is next command
    // check if need to send sensor reading
    if(sendSensorReading){
      obstacleIdentification();
      sendSensorReading = false;
      
      // if obstacle position is x1x1x, reposition robot
      if(obstaclePositions[1]==1 && obstaclePositions[3]==1)
        repositionRobotFront();
      
    }
    // check for next command
    if(commandBuffer.length() > 1){
      command = commandBuffer.charAt(0)-48;
      commandBuffer = commandBuffer.substring(1);
    }
    else if(commandBuffer.length() == 1){
      command = commandBuffer.charAt(0)-48;
      commandBuffer = "";
    }
    else command = 0;
    
    //Serial.println(commandBuffer);
    break;
    
    case 1: // move 1 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      for(unsigned long movement_start_time = millis(); (millis() - movement_start_time)<_1_grid_movement_duration;)
        moveForward();
  
      analogWrite(m1PWM, 0*255);
      analogWrite(m2PWM, 0*255);
      command = 0;
      // send ACK back to Raspberry pi
      //Serial.println("ACK");
      sendSensorReading = true;
    break;
    
    case 2: // rotate left 90
      rotateLeft90();
      command = 0;
      // send ACK back to Raspberry pi
      //Serial.println("ACK");
      sendSensorReading = true;
    break; 
    
    case 3: // rotate right 90
      rotateRight90();
      command = 0;
      // send ACK back to Raspberry pi
      //Serial.println("ACK");
      sendSensorReading = true;
    break;
    
    case 4: // left 180
      rotateLeft180();
      command = 0;
      // send ACK back to Raspberry pi
      //Serial.println("ACK");
      sendSensorReading = true;
    break;
    
    case 5:
    repositionRobotFront();
    command = 0;
    break;
    default: command = 0; // go to idle state by default
  }
}

void serialEvent(){
  // get data from serial buffer
  while(Serial.available()){
    char inChar = (char)Serial.read();
    commandBuffer += inChar;
  }
  // remove whitespaces
  commandBuffer.trim();
}

void obstacleIdentification(){
  for(long startTime=millis();(millis()-startTime)<sensor_cooldown_duration;){
    computeMedian();
    sonarReading();
  }
  // check left side distance
  if(leftSideSensorMedian>=21 && leftSideSensorMedian<=26)
    obstaclePositions[0] = 3;
  else if(leftSideSensorMedian>=12 && leftSideSensorMedian<=20)
    obstaclePositions[0] = 2;
  else if(leftSideSensorMedian<12 && leftSideSensorMedian>0)
    obstaclePositions[0] = 1;
  else 
    obstaclePositions[0] = 0;
    
  // check left diag distance
  if(leftDiagSensorMedian>=18 && leftDiagSensorMedian<=30)
    obstaclePositions[1] = 3;
  else if(leftDiagSensorMedian>=13 && leftDiagSensorMedian<=17)
    obstaclePositions[1] = 2;
  else if(leftDiagSensorMedian<12 && leftDiagSensorMedian>0)
    obstaclePositions[1] = 1;
  else 
    obstaclePositions[1] = 0;
    
  // check right diag distance
  if(rightDiagSensorMedian>=19 && rightDiagSensorMedian<=30)
    obstaclePositions[3] = 3;
  else if(rightDiagSensorMedian>=13 && rightDiagSensorMedian<=18)
    obstaclePositions[3] = 2;
  else if(rightDiagSensorMedian<12 && rightDiagSensorMedian>0)
    obstaclePositions[3] = 1;
  else
    obstaclePositions[3] = 0;
  
  // check ultrasonic distance
  if(sonarDist>=23 && sonarDist<=30)
    obstaclePositions[2] = 3;
  else if(sonarDist>=12 && sonarDist<=16)
    obstaclePositions[2] = 2;
  else if(sonarDist<=11)
    obstaclePositions[2] = 1;
  else 
    obstaclePositions[2] = 0;
    
  // check right side distance
  if(rightSideSensorMedian>=22 && rightSideSensorMedian<=30)
    obstaclePositions[4] = 3;
  else if(rightSideSensorMedian>=13 && rightSideSensorMedian<=21)
    obstaclePositions[4] = 2;
  else if(rightSideSensorMedian<12 && rightSideSensorMedian>0)
    obstaclePositions[4] = 1;
  else 
    obstaclePositions[4] = 0;
  Serial.print(obstaclePositions[0]);
  Serial.print(obstaclePositions[1]);
  Serial.print(obstaclePositions[2]);
  Serial.print(obstaclePositions[3]);
  Serial.println(obstaclePositions[4]);
  //Serial.println("----");
}

int leftSideSensorReading(){
 return ((5000/(analogRead(4)-10))-3); 
}

int leftDiagSensorReading(){
 return ((5200/(analogRead(2)-10))-3); 
}

int rightDiagSensorReading(){
 return ((5210/(analogRead(3)-10))-3); 
}

int rightSideSensorReading(){
 return ((5000/(analogRead(5)-10))-3); 
}

void computeMedian(){
  static int sideIndex=0;
  static int diagIndex=0;
  // get reading
  leftSideSensor[sideIndex] = leftSideSensorReading();
  leftDiagSensor[diagIndex] = leftDiagSensorReading();
  rightSideSensor[sideIndex] = rightSideSensorReading();
  rightDiagSensor[diagIndex] = rightDiagSensorReading();
  // sort data     
  insertionSort();
  
  leftSideSensorMedian = leftSideSensor[samples/2];
  leftDiagSensorMedian = leftDiagSensor[samples/2];
  rightSideSensorMedian = rightSideSensor[samples/2];
  rightDiagSensorMedian = rightDiagSensor[samples/2];
  
  sideIndex = (sideIndex+1)%samples;
  diagIndex = (diagIndex+1)%samples;
}

void insertionSort(){
  for(int i=0; i<samples; i++){
    for(int j=i; j>0;j--){
      if(leftSideSensor[j]<leftSideSensor[j-1]){
        int temp = leftSideSensor[j];
        leftSideSensor[j] = leftSideSensor[j-1];
        leftSideSensor[j-1] = temp;
      }
      if(leftDiagSensor[j]<leftDiagSensor[j-1]){
        int temp = leftDiagSensor[j];
        leftDiagSensor[j] = leftDiagSensor[j-1];
        leftDiagSensor[j-1] = temp;
      }
      if(rightSideSensor[j]<rightSideSensor[j-1]){
        int temp = rightSideSensor[j];
        rightSideSensor[j] = rightSideSensor[j-1];
        rightSideSensor[j-1] = temp;
      }
      if(rightDiagSensor[j]<rightDiagSensor[j-1]){
        int temp = rightDiagSensor[j];
        rightDiagSensor[j] = rightDiagSensor[j-1];
        rightDiagSensor[j-1] = temp;
      }
    }
  } 
}

void sonarReading(){
  unsigned long pulseDuration = (pulseIn(URPWM, LOW, 12000));
  if(pulseDuration<=50000 && pulseDuration>0)
    sonarDist = pulseDuration/50; // 1cm for every 50us
}

void moveForward(){
  // Setting wheels to move
  analogWrite(m1PWM, m1DC*255);
  analogWrite(m2PWM, m2DC*255);
  
  unsigned long current_ms = millis();
  // Compute FSM to track rate of rising edges per 0.4 sec from encoder output
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

void rotateLeft90(){
  // setting wheel direction to rotate robot clockwise
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM, 0.2*255);
  analogWrite(m1PWM, 0.2*255);
  
  delay(leftRotationTime);
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  delay(200);
  analogWrite(m2PWM, 0.2*255);
  analogWrite(m1PWM, 0.2*255);
  delay(leftRotationAdjustmentTime);
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void rotateLeft180(){
  // setting wheel direction to rotate robot clockwise
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM, 0.2*255);
  analogWrite(m1PWM, 0.2*255);
  
  delay(left180RotationTime);
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void rotateRight90(){
  // setting wheel direction to rotate robot clockwise
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, LOW);
  
  analogWrite(m2PWM, 0.2*255);
  analogWrite(m1PWM, 0.2*255);
  
  delay(rightRotationTime);
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  delay(200);
  analogWrite(m2PWM, 0.2*255);
  analogWrite(m1PWM, 0.2*255);
  delay(rightRotationAdjustmentTime);
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void repositionRobotFront(){
  // compute median sensor readings to know if robot is facing straight
  for(long start_time = millis(); (millis()-start_time)<sensor_cooldown_duration;)
    computeMedian();
  if(leftDiagSensorMedian>rightDiagSensorMedian){  
    // set wheels to rotate right
    digitalWrite(m1INB, LOW);
    digitalWrite(m2INB, HIGH);
    digitalWrite(m1INA, HIGH);
    digitalWrite(m2INA, LOW);
    
    // start rotating
    analogWrite(m2PWM, 0.15*255);
    analogWrite(m1PWM, 0.15*255);
    
    while(leftDiagSensorMedian>rightDiagSensorMedian)
      computeMedian();
  } else if(leftDiagSensorMedian<rightDiagSensorMedian){
    // set wheels to rotate left
    digitalWrite(m1INB, HIGH);
    digitalWrite(m2INB, LOW);
    digitalWrite(m1INA, LOW);
    digitalWrite(m2INA, HIGH);
    // start rotating
    analogWrite(m2PWM, 0.16*255);
    analogWrite(m1PWM, 0.16*255);
    
    while(leftDiagSensorMedian<rightDiagSensorMedian)
      computeMedian();
  }
  analogWrite(m2PWM, 0);
  analogWrite(m1PWM, 0);
}
