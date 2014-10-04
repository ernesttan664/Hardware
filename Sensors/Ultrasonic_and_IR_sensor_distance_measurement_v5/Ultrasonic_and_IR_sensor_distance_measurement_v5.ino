/* 
 Arduino          Sensor
 Vcc              Pin 1 VCC (URM V3.2)
 GND              Pin 2 GND (URM V3.2)
 Pin 11           Pin 4 PWM (URM V3.2)
 Pin 13           Pin 6 COMP/TRIG (URM V3.2)
 Pin A2           Left Diagonal IR Sensor
 Pin A3           Right Diag IR Sensor
 Pin A4           Left Side IR Sensor
 Pin A5           Right Side IR Sensor
 
 Working Mode: autonomous  mode.
 Timeout for pulseIn set to 4025us so the range of the US sensor is limited to an effective 80cm
 During tuning of PID parameters, will need to take into account this lag period
  
 
*/
#define samples 21
#define median_size 100

int URPWM = 11; // PWM Output 0-25000US,Every 50US represent 1cm
int URCOMP= 13; // PWM trigger pin
long prev_sensor_refresh_time=0;
long refresh_time_interval = 0;
int leftSideSensor[samples], leftDiagSensor[samples], rightSideSensor[samples], rightDiagSensor[samples];
int left[median_size], leftDiag[median_size], right[median_size], rightDiag[median_size];
int leftSideSensorMedian=0, leftDiagSensorMedian=0, rightDiagSensorMedian=0, rightSideSensorMedian=0;
static int sonarDist=0;
int leftMin=9999, leftDiagMin=9999, rightMin=9999, rightDiagMin=9999;
int leftMax=0, leftDiagMax=0, rightMax=0, rightDiagMax=0;
String obstaclePositions = "";

void setup() 
{
  //Serial initialization
  Serial.begin(9600);
  
  for(int i=0; i<samples; i++){
    leftSideSensor[i]=999;
    leftDiagSensor[i]=999;
    rightSideSensor[i]=999;
    rightDiagSensor[i]=999;
    
  }
  Serial.println("Setup complete");
 }

void loop(){  
  //sensorDataRange();
  getSensorReadings();
  obstacleIdentification();
  
}

void obstacleIdentification(){
  refresh_time_interval=1000;
  for(long startTime=millis();(millis()-startTime)<refresh_time_interval;){
    computeMedian();
    sonarReading();
  }
  // check 3 grids away
  if(leftDiagSensorMedian>=18 && leftDiagSensorMedian<=28)
    obstaclePositions += "3";
  else 
    obstaclePositions += "0";
  if(sonarDist>=23 && sonarDist<=25)
    obstaclePositions += "3";
  else 
    obstaclePositions += "0";
  if(rightDiagSensorMedian>=17 && rightDiagSensorMedian<=28)
    obstaclePositions += "3";
  else 
    obstaclePositions += "0";
  obstaclePositions += "\n";
  
  // check 2 grids away
  if(leftDiagSensorMedian>=12 && leftDiagSensorMedian<=16)
    obstaclePositions += "2";
  else 
    obstaclePositions += "0";
  if(sonarDist>=14 && sonarDist<=15)
    obstaclePositions += "2";
  else 
    obstaclePositions += "0";
  if(rightDiagSensorMedian>=13 && rightDiagSensorMedian<=16)
    obstaclePositions += "2";
  else 
    obstaclePositions += "0";
  obstaclePositions += "\n";
  
  // check 1 grid away
  if(leftDiagSensorMedian>=5 && leftDiagSensorMedian<=6)
    obstaclePositions += "1";
  else 
    obstaclePositions += "0";  
  if(sonarDist>=5 && sonarDist<=6)
    obstaclePositions += "1";
  else 
    obstaclePositions += "0";
  if(rightDiagSensorMedian>=5 && rightDiagSensorMedian<=6)
    obstaclePositions += "1";
  else 
    obstaclePositions += "0";
  obstaclePositions += "\n";
  
  Serial.print(obstaclePositions);
  Serial.println("----");
  obstaclePositions = "";
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

void sensorDataRange(){
  for(int i=0; i<median_size; i++){
    computeMedian();
    left[i] = leftSideSensorMedian;
    leftDiag[i] = leftDiagSensorMedian;
    right[i] = rightSideSensorMedian;
    rightDiag[i] = rightDiagSensorMedian;
  }
  if(leftMin > left[0])
    leftMin = left[0];
  if(leftMax < left[median_size-1])
   leftMax = left[median_size-1];
  
  if(leftDiagMin > leftDiag[0])
    leftDiagMin = leftDiag[0];
  if(leftDiagMax < leftDiag[median_size-1])
   leftDiagMax = leftDiag[median_size-1];
  
  if(rightMin > right[0])
    rightMin = right[0];
  if(rightMax < right[median_size-1])
   rightMax = right[median_size-1];
  
  if(rightDiagMin > rightDiag[0])
    rightDiagMin = rightDiag[0];
  if(rightDiagMax < rightDiag[median_size-1])
   rightDiagMax = rightDiag[median_size-1];
  sonarReading();
  
  long current_time=millis();
  if(current_time - prev_sensor_refresh_time > refresh_time_interval){
    //Serial.println("Left\tLeft Diag\t\tUltrasonic\tRight Diag\t\tRight");
    Serial.print(leftMin);
    Serial.print(" to ");
    Serial.print(leftMax);
    Serial.print("\t");
    
    Serial.print(leftDiagMin);
    Serial.print(" to ");
    Serial.print(leftDiagMax);
    Serial.print("\t\t");
    
    Serial.print(sonarDist);
    Serial.print("\t\t");
    
    Serial.print(rightDiagMin);
    Serial.print(" to ");
    Serial.print(rightDiagMax);
    Serial.print("\t");
    
    Serial.print(rightMin);
    Serial.print(" to ");
    Serial.print(rightMax);
    Serial.println("\t\t");
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

void serialEvent(){
  int data = Serial.read();
  Serial.println("Resetting max/min values");
  //String command = String(data);
  if(data == 49){
    leftMin=9999;
    leftDiagMin=9999;
    rightMin=9999;
    rightDiagMin=9999;
    leftMax=0;
    leftDiagMax=0;
    rightMax=0;
    rightDiagMax=0;
  }
}


