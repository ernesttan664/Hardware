/*
  Left motor = M2, Right motor = M1

  Connections             Arduino Pin
  M2INA output --->       pin 7
  M1INA output --->       pin 2
  Vcc              Pin 1 VCC (URM V3.2)
  GND              Pin 2 GND (URM V3.2)
  Pin 11           Pin 4 PWM (URM V3.2)
  Pin 13           Pin 6 COMP/TRIG (URM V3.2)
  Pin A2           Left Diagonal IR Sensor
  Pin A3           Right Diag IR Sensor
  Pin A4           Left Side IR Sensor
  Pin A5           Right Side IR Sensor

*/
#define samples 21
#define median_size 100

int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int URPWM = 11; // PWM Output 0-25000US,Every 50US represent 1cm
int URCOMP= 13; // PWM trigger pin
long prev_sensor_refresh_time=0;
long refresh_time_interval = 1000;
int leftSideSensor[samples], leftDiagSensor[samples], rightSideSensor[samples], rightDiagSensor[samples];
int left[median_size], leftDiag[median_size], right[median_size], rightDiag[median_size];
int leftSideSensorMedian=0, leftDiagSensorMedian=0, rightDiagSensorMedian=0, rightSideSensorMedian=0;
static int sonarDist=0;

void setup(){
  Serial.begin(9600);
  Serial.println("Setup complete");
}

void loop(){
  repositionRobot();
  delay(1000);
  
}

void repositionRobot(){
  // compute median sensor readings to know if robot is facing straight
  for(long start_time = millis(); (millis()-start_time)<refresh_time_interval;)
    computeMedian();
  Serial.println("reposition");
  Serial.print(leftDiagSensorMedian);
  Serial.print(" ");
  Serial.print(rightDiagSensorMedian);
  if(leftDiagSensorMedian>rightDiagSensorMedian){
    Serial.println("rotate right");
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
    Serial.println("stop");
    analogWrite(m2PWM, 0);
    analogWrite(m1PWM, 0);
  } else if(leftDiagSensorMedian<rightDiagSensorMedian){
     Serial.println("rotate left");
    // set wheels to rotate left
    digitalWrite(m1INB, HIGH);
    digitalWrite(m2INB, LOW);
    digitalWrite(m1INA, LOW);
    digitalWrite(m2INA, HIGH);
    // start rotating
    analogWrite(m2PWM, 0.15*255);
    analogWrite(m1PWM, 0.15*255);
    
    while(leftDiagSensorMedian<rightDiagSensorMedian)
      computeMedian();
    Serial.println("stop");
    analogWrite(m2PWM, 0);
    analogWrite(m1PWM, 0);
  }
  
}

void getSensorReadings(){
  long current_time=millis();
  computeMedian();
  if(current_time - prev_sensor_refresh_time > refresh_time_interval){
    Serial.print(leftSideSensorMedian);
    Serial.print(" ");
    Serial.print(leftDiagSensorMedian);
    Serial.print(" ");
    sonarReading();
    Serial.print(sonarDist);
    Serial.print(" ");
    Serial.print(rightDiagSensorMedian);
    Serial.print(" ");
    Serial.println(rightSideSensorMedian);
    prev_sensor_refresh_time = current_time;
  }
}

int leftSideSensorReading(){
 return ((6000/(analogRead(4)-3))-3); 
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
    sonarDist = pulseDuration/50; // 1cm for every 50us low level stands for
}
