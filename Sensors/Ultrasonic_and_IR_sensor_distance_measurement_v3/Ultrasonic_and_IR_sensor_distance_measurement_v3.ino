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

int URPWM = 11; // PWM Output 0-25000US,Every 50US represent 1cm
int URCOMP= 13; // PWM trigger pin
static int sonarDist=0;
void setup() 
{
  //Serial initialization
  Serial.begin(9600);
  Serial.println("Setup complete");
 }

void loop()
{
  Serial.print(leftSideSensorReading());
  Serial.print(" ");
  Serial.print(leftDiagSensorReading());
  Serial.print(" ");
  sonarReading();
  Serial.print(sonarDist);
  Serial.print(" ");
  Serial.print(rightDiagSensorReading());
  Serial.print(" ");
  Serial.println(rightSideSensorReading());
  delay(100);
} 

int leftSideSensorReading(){
 return ((7100/(analogRead(4)-3))-3); 
}

int leftDiagSensorReading(){
 return ((5000/(analogRead(2)-10))); 
}

int rightDiagSensorReading(){
 return ((5500/(analogRead(3)-10))-3); 
}

int rightSideSensorReading(){
 return ((4800/(analogRead(5)-35))-2); 
}

void sonarReading(){
  unsigned long pulseDuration = (pulseIn(URPWM, LOW, 10000));
  if(pulseDuration<=50000 && pulseDuration>0)
    sonarDist = pulseDuration/50; // 1cm for every 50us low level stands for 
}



