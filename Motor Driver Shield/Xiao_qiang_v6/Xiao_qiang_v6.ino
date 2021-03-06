/*
  Left motor = M2, Right motor = M1
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5
  
  Robot Communications Protocol v3.0
    Whenever a command finishes execution, sensor data is relayed back at the end of each movement command
  
  At software lab 3 arena, speed = 500, interval = 100
  m2_P = 1.1, m2_I = 0.0001, m2_D = 0.001;
  m1_P = 1.06, m1_I = 0.0001, m1_D = 0.001;
  
  need to reduce amount of positioning, can try to reposition every 2 to 3 moves
  priority: reduce sensor cooldown time

*/
#include <PinChangeInt.h>
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
unsigned long prev_ms = 0, sensor_cooldown_duration = 400;
unsigned long interval = 100 ; // time unit = ms
unsigned long _1_grid_movement_duration = 450; 
unsigned long _2_grid_movement_duration = 900; 
unsigned long _3_grid_movement_duration = 1250;
unsigned long _10_grid_movement_duration = 4200;
int command=0, sonarDist=0, avgCount=0;
double m2DC = 0.0, m1DC = 0.0,m1Count=0.0, m2Count=0.0, m1SpeedAdjustment, m2SpeedAdjustment, targetSpeed = 0.0; // note: adjust target speed at the case statements
volatile int m2MovementCount=0, m1MovementCount=0;
double m2_P = 1.1, m2_I = 0.0001, m2_D = 0.001;
double m1_P = 1.05, m1_I = 0.0001, m1_D = 0.001;
String commandBuffer = "111115111114"; // 71277776377627
boolean sendSensorReading=true, explorationMode=true;
int leftSideSensor[samples], leftDiagSensor[samples], rightSideSensor[samples], rightDiagSensor[samples], adjustmentSensor[samples];
int leftSideSensorMedian=0, leftDiagSensorMedian=0, rightDiagSensorMedian=0, rightSideSensorMedian=0, adjustmentSensorMedian=0;
int obstaclePositions[6];

//Specify the links and initial tuning parameters
PID leftPID(&m2Count, &m2SpeedAdjustment, &targetSpeed, m2_P, m2_I , m2_D, DIRECT);
PID rightPID(&m1Count, &m1SpeedAdjustment, &targetSpeed, m1_P, m1_I, m1_D, DIRECT);


void setup() {
  // setting String buffer size 
  commandBuffer.reserve(200);
  
  //tell the PID to range between -88 and 800
  leftPID.SetOutputLimits(-800,800);
  rightPID.SetOutputLimits(-800,800);
  
  //turn the PID on
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
  // setup pin interrupts
  PCintPort::attachInterrupt(PIN3, &compute_m2_count, RISING);
  PCintPort::attachInterrupt(PIN5, &compute_m1_count, RISING);
  Serial.begin(115200); // set the baud rate
  
  // Establishing Communications
  while(!Serial);
  
  // waiting for ASCII 0 to be sent from pc to indicate start of exploration
  //while(Serial.available()<1){}
}

void loop(){
  switch(command){
    case 0:  // idle state, send readings once and check if there is next command
      // check if need to send sensor reading
      if(sendSensorReading && explorationMode){
        obstacleIdentification();
        sendSensorReading = false;
        
        // if obstacle position is x1x1x, reposition robot
        if(obstaclePositions[1]==1 && obstaclePositions[3]==1){
          repositionRobotFront();
        }
        // if side of the robot is next to a wall
        if(obstaclePositions[4]==1 && obstaclePositions[5]==1){
          repositionRobotSide();
        }
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
      delay(100);
    break;
    
    case 1: // move 1 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      targetSpeed = 500;
      for(m2MovementCount=0; m2MovementCount<160;)
        moveForward();
      targetSpeed = 200;
      for(m2MovementCount=0; m2MovementCount<95;)
        moveForward();
      analogWrite(m2PWM, 0*255);
      analogWrite(m1PWM, 0*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 2: // rotate left 90
      rotateLeft90();
      command = 0;
      sendSensorReading = true;
    break; 
    
    case 3: // rotate right 90
      rotateRight90();
      command = 0;
      sendSensorReading = true;
    break;
    
    case 4: // left 180
      rotateLeft180();
      command = 0;
      sendSensorReading = true;
    break;
    
    case 5: // right 180
      rotateRight180();
      command = 0;
      sendSensorReading = true;
    break;
    
    case 6: //  move 2 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      targetSpeed = 500;
      for(unsigned long movement_start_time = millis(); (millis() - movement_start_time)<_2_grid_movement_duration;)
        moveForward();
      analogWrite(m2PWM, 0*255);
      analogWrite(m1PWM, 0*255);
      sendSensorReading = true;
      command = 0;
    break;
    
    case 7: // move 3 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      targetSpeed = 500;
      
      for(unsigned long movement_start_time = millis(); (millis() - movement_start_time)<_3_grid_movement_duration;)
        moveForward();
      analogWrite(m2PWM, 0*255);
      analogWrite(m1PWM, 0*255);
      sendSensorReading = true;
      command = 0;
    break;
    
    case 8: // move 10 grid
    // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      targetSpeed = 500;
      for(unsigned long movement_start_time = millis(); (millis() - movement_start_time)<_10_grid_movement_duration;)
        moveForward();
      analogWrite(m1PWM, 0*255);
      analogWrite(m2PWM, 0*255);
      sendSensorReading = true;
      command = 0;
    break;
    
    case 17: // realign robot centre to be within grid, command = "A"
      repositionRobotFront();
      realignRobotCentre();
      repositionRobotFront();
      sendSensorReading = true;
      command = 0;
    break;
    
    case 18: // realign robot centre to make sure it is within 3x3 grid, command = "B"
      rotateRight90();
      repositionRobotFront();
      realignRobotCentre();
      repositionRobotFront();
      rotateLeft90();
      repositionRobotSide();
      sendSensorReading = true;
      command = 0;
    break;
    default: command = 0; // go to idle state by default
  }
}

void serialEvent(){
  // get data from serial buffer
  while(Serial.available()){
    char inChar = (char)Serial.read();
    if(inChar != 's')  // s indicate start of shortest path run, sensor readings will not be computed 
      commandBuffer += inChar;
    else
      explorationMode = false;
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
  if(leftSideSensorMedian>=17 && leftSideSensorMedian<=25)
    obstaclePositions[0] = 3;
  else if(leftSideSensorMedian>=7 && leftSideSensorMedian<=16)
    obstaclePositions[0] = 2;
  else if(leftSideSensorMedian<7 && leftSideSensorMedian>0)
    obstaclePositions[0] = 1;
  else 
    obstaclePositions[0] = 0;
    
  // check left diag distance
  if(leftDiagSensorMedian>=18 && leftDiagSensorMedian<=27)
    obstaclePositions[1] = 3;
  else if(leftDiagSensorMedian>=11 && leftDiagSensorMedian<=17)
    obstaclePositions[1] = 2;
  else if(leftDiagSensorMedian<11 && leftDiagSensorMedian>0)
    obstaclePositions[1] = 1;
  else 
    obstaclePositions[1] = 0;
    
  // check right diag distance
  if(rightDiagSensorMedian>=19 && rightDiagSensorMedian<=31)
    obstaclePositions[3] = 3;
  else if(rightDiagSensorMedian>=11 && rightDiagSensorMedian<=18)
    obstaclePositions[3] = 2;
  else if(rightDiagSensorMedian<11 && rightDiagSensorMedian>0)
    obstaclePositions[3] = 1;
  else
    obstaclePositions[3] = 0;
  
  // check ultrasonic distance
  if(sonarDist>16 && sonarDist<=30)
    obstaclePositions[2] = 3;
  else if(sonarDist>=8 && sonarDist<=16)
    obstaclePositions[2] = 2;
  else if(sonarDist<=7)
    obstaclePositions[2] = 1;
  else 
    obstaclePositions[2] = 0;
    
  // check right side distance
  if(rightSideSensorMedian>=21 && rightSideSensorMedian<=24)
    obstaclePositions[4] = 3;
  else if(rightSideSensorMedian>=7 && rightSideSensorMedian<=20)
    obstaclePositions[4] = 2;
  else if(rightSideSensorMedian<7 && rightSideSensorMedian>0)
    obstaclePositions[4] = 1;
  else 
    obstaclePositions[4] = 0;
    
  // check adjustment sensor distance
  if(adjustmentSensorMedian>=17 && adjustmentSensorMedian<=20)
    obstaclePositions[5] = 3;
  else if(adjustmentSensorMedian>=9 && adjustmentSensorMedian<=16)
    obstaclePositions[5] = 2;
  else if(adjustmentSensorMedian<9 && adjustmentSensorMedian>0)
    obstaclePositions[5] = 1;
  else 
    obstaclePositions[5] = 0;
  Serial.print("pc:");
  Serial.print(obstaclePositions[0]);
  Serial.print(obstaclePositions[1]);
  Serial.print(obstaclePositions[2]);
  Serial.print(obstaclePositions[3]);
  Serial.println(obstaclePositions[4]);
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

int adjustmentSensorReading(){
 return ((4400/(analogRead(0)+5))-3); 
}

void computeMedian(){
  static int sideIndex=0;
  static int diagIndex=0;
  // get reading
  leftSideSensor[sideIndex] = leftSideSensorReading();
  leftDiagSensor[diagIndex] = leftDiagSensorReading();
  rightSideSensor[sideIndex] = rightSideSensorReading();
  rightDiagSensor[diagIndex] = rightDiagSensorReading();
  adjustmentSensor[diagIndex] = adjustmentSensorReading();
  // sort data     
  insertionSort();
  
  leftSideSensorMedian = leftSideSensor[samples/2];
  leftDiagSensorMedian = leftDiagSensor[samples/2];
  rightSideSensorMedian = rightSideSensor[samples/2];
  rightDiagSensorMedian = rightDiagSensor[samples/2];
  adjustmentSensorMedian = adjustmentSensor[samples/2];
  
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
      if(adjustmentSensor[j]<adjustmentSensor[j-1]){
        int temp = adjustmentSensor[j];
        adjustmentSensor[j] = adjustmentSensor[j-1];
        adjustmentSensor[j-1] = temp;
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

void compute_m2_count(){
  m2Count++;
  m2MovementCount++;
}

void compute_m1_count(){
  m1Count++;
}

void rotateLeft90(){
  m2Count=0;
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
  while(m2Count<440){
    delay(1);
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  m2Count=0;
}

void rotateLeft180(){
  m2Count=0;
  /// setting wheel direction to rotate robot left
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(900);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(m2Count<890){
    delay(1);
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  m2Count=0;
}

void rotateRight180(){
  m2Count=0;
  /// setting wheel direction to rotate robot left
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, LOW);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(900);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(m2Count<810){
    delay(1);
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  m2Count=0;
}

void rotateRight90(){
  m2Count=0;
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
  while(m2Count<405){
    delay(1);
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
  m2Count=0;
}

void repositionRobotFront(){
  // compute median sensor readings to know if robot is facing straight
//  for(long start_time = millis(); (millis()-start_time)<sensor_cooldown_duration;)
//    computeMedian();
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

void repositionRobotSide(){
  // compute median sensor readings to know if robot is facing straight
//  for(long start_time = millis(); (millis()-start_time)<(sensor_cooldown_duration+200);)
//    computeMedian();
  if(rightSideSensorMedian>adjustmentSensorMedian){  
    // set wheels to rotate left
    digitalWrite(m1INB, HIGH);
    digitalWrite(m2INB, LOW);
    digitalWrite(m1INA, LOW);
    digitalWrite(m2INA, HIGH);
    // start rotating
    analogWrite(m2PWM, 0.175*255);
    analogWrite(m1PWM, 0.175*255);
    
    while(rightSideSensorMedian>adjustmentSensorMedian)
      computeMedian();
  } else if(rightSideSensorMedian<adjustmentSensorMedian){
    // set wheels to rotate right
    digitalWrite(m1INB, LOW);
    digitalWrite(m2INB, HIGH);
    digitalWrite(m1INA, HIGH);
    digitalWrite(m2INA, LOW);
    // start rotating
    analogWrite(m2PWM, 0.175*255);
    analogWrite(m1PWM, 0.175*255);
    
    while(rightSideSensorMedian<adjustmentSensorMedian)
      computeMedian();
  }
  analogWrite(m2PWM, 0);
  analogWrite(m1PWM, 0);
}

void realignRobotCentre(){
  // get sonar dist
  sonarReading();
  if(sonarDist!=4){
    if(sonarDist>4){
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
    }
    else if(sonarDist<4){
      // Setting wheels to move robot forward
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, HIGH);
      digitalWrite(m2INA, HIGH);
    }
    analogWrite(m2PWM, 0.2*255);
    analogWrite(m1PWM, 0.2*255);
    while(sonarDist>4)
      sonarReading();
    analogWrite(m2PWM, 0);
    analogWrite(m1PWM, 0);
  }
}
