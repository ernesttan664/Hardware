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
 
  need 180deg turn  
 
*/
#define samples 5

int URPWM = 11; // PWM Output 0-25000US,Every 50US represent 1cm
int URCOMP= 13; // PWM trigger pin
long prev_sensor_refresh_time = 0;
long refresh_time_interval = 500;
int leftSideSensor[samples], leftDiagSensor[samples], rightSideSensor[samples], rightDiagSensor[samples];
int sideIndex=0, diagIndex=0;
double leftSideTotal = 0.0, leftDiagTotal = 0.0, rightSideTotal = 0.0, rightDiagTotal = 0.0;
double leftSideAvg=0.0, leftDiagAvg=0.0, rightSideAvg=0.0, rightDiagAvg=0.0;
static int sonarDist=0;

void setup() 
{
  //Serial initialization
  Serial.begin(9600);
  
  for(int i=0; i<samples; i++)
    leftSideSensor[i]=0;
  Serial.println("Setup complete");
 }

void loop()
{
  
  long current_time=millis();
  computeMovingAverage();
  if(current_time - prev_sensor_refresh_time > refresh_time_interval){
    Serial.print(leftSideAvg);
    Serial.print(" ");
    Serial.print(leftDiagAvg);
    Serial.print(" ");
    sonarReading();
    Serial.print(sonarDist);
    Serial.print(" ");
    Serial.print(rightDiagAvg);
    Serial.print(" ");
    Serial.println(rightSideAvg);
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

void computeMovingAverage(){
  // subtract the last reading:
  leftSideTotal -= leftSideSensor[sideIndex];
  leftDiagTotal -= leftDiagSensor[sideIndex];
  rightSideTotal -= rightSideSensor[sideIndex];
  rightDiagTotal -= rightDiagSensor[sideIndex];
  
  // read from the sensor:  
  leftSideSensor[sideIndex] = leftSideSensorReading(); 
  leftDiagSensor[sideIndex] = leftDiagSensorReading(); 
  rightSideSensor[sideIndex] = rightSideSensorReading(); 
  rightDiagSensor[sideIndex] = rightDiagSensorReading();
   
   // add the reading to the total:
   leftSideTotal += leftSideSensor[sideIndex];    
   leftDiagTotal += leftDiagSensor[sideIndex];
   rightSideTotal += rightSideSensor[sideIndex];
   rightDiagTotal += rightDiagSensor[sideIndex];

   // advance to the next position in the array:  
   sideIndex++;                    
   diagIndex++;
  
   if ((sideIndex >= samples)||(diagIndex >= samples)){
     sideIndex = 0;       
     diagIndex = 0;     
   }
   // calculate the average:
   leftSideAvg = leftSideTotal / samples;
   leftDiagAvg = leftDiagTotal / samples;
   rightSideAvg = rightSideTotal / samples;
   rightDiagAvg = rightDiagTotal / samples;
}

void sonarReading(){
  unsigned long pulseDuration = (pulseIn(URPWM, LOW, 10000));
  if(pulseDuration<=50000 && pulseDuration>0)
    sonarDist = pulseDuration/50; // 1cm for every 50us low level stands for 
}



