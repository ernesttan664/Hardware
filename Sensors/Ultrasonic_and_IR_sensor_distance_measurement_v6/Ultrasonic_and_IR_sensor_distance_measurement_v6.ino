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
int leftSideSensor[samples], leftDiagSensor[samples], rightSideSensor[samples], rightDiagSensor[samples], adjustmentSensor[samples],leftAdjustmentSensor[samples];
int left[median_size], leftDiag[median_size], right[median_size], rightDiag[median_size], adjustment[median_size];
int leftSideSensorMedian=0, leftDiagSensorMedian=0, rightDiagSensorMedian=0, rightSideSensorMedian=0, adjustmentSensorMedian=0, leftAdjustmentSensorMedian=0;
static int sonarDist=0;
int leftMin=9999, leftDiagMin=9999, rightMin=9999, rightDiagMin=9999;
int leftMax=0, leftDiagMax=0, rightMax=0, rightDiagMax=0;
int obstaclePositions[7];

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
  //Serial.println(adjustmentSensorReading());
  //delay(100);
}

void obstacleIdentification(){
  refresh_time_interval=400;
  for(long startTime=millis();(millis()-startTime)<refresh_time_interval;){
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
  // check adjustment sensor distance
  if(leftAdjustmentSensorMedian>=17 && leftAdjustmentSensorMedian<=20)
    obstaclePositions[6] = 3;
  else if(leftAdjustmentSensorMedian>=9 && leftAdjustmentSensorMedian<=16)
    obstaclePositions[6] = 2;
  else if(leftAdjustmentSensorMedian<9 && leftAdjustmentSensorMedian>0)
    obstaclePositions[6] = 1;
  else 
    obstaclePositions[6] = 0;
  Serial.print("pc:");
  Serial.print(obstaclePositions[0]);
  Serial.print(obstaclePositions[1]);
  Serial.print(obstaclePositions[2]);
  Serial.print(obstaclePositions[3]);
  Serial.print(obstaclePositions[4]);
  Serial.print(obstaclePositions[5]);
  Serial.println(obstaclePositions[6]);
  Serial.println("----");
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
    Serial.print(rightSideSensorMedian);
    Serial.print(" ");
    Serial.print(adjustmentSensorMedian);
    Serial.print(" ");
    Serial.println(leftAdjustmentSensorMedian);
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

int leftAdjustmentSensorReading(){
 return ((4600/(analogRead(1)+5))-3); 
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
  leftAdjustmentSensor[diagIndex] = leftAdjustmentSensorReading();
  // sort data     
  insertionSort();
  
  leftSideSensorMedian = leftSideSensor[samples/2];
  leftDiagSensorMedian = leftDiagSensor[samples/2];
  rightSideSensorMedian = rightSideSensor[samples/2];
  rightDiagSensorMedian = rightDiagSensor[samples/2];
  adjustmentSensorMedian = adjustmentSensor[samples/2];
  leftAdjustmentSensorMedian = leftAdjustmentSensor[samples/2];
  
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
      if(leftAdjustmentSensor[j]<leftAdjustmentSensor[j-1]){
        int temp = leftAdjustmentSensor[j];
        leftAdjustmentSensor[j] = leftAdjustmentSensor[j-1];
        leftAdjustmentSensor[j-1] = temp;
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


