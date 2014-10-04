#include <PID_v1.h>

// Initialization
unsigned long prevMicroSec = 0;
unsigned long interval = 1000000; // time unit = us
double input = 0.0, Setpoint, Output;
double kp = 1, ki = 0.01, kd = 0.02;
//double kp = 0.019208, ki = 0.0010034, kd = 0.0;

//Specify the links and initial tuning parameters
PID myPID(&input, &Output, &Setpoint, kp, ki , kd, DIRECT);

void setup(){  
  // Starting Connections
  Serial.begin(9600);
  // Initializing target value
  Setpoint = 10.0;
  
  //tell the PID to range between 0 and max
  myPID.SetOutputLimits(-2,2);
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
    
  Serial.println("Setup Complete");
  
}

void loop(){
  
  unsigned long currentMicroSec = micros();
  
  if((currentMicroSec - prevMicroSec) > interval){
    
    // Compute PID
    myPID.Compute();
    input += Output;
    Serial.print("Input = ");
    Serial.print(input);
    Serial.print(" Output =  ");
    Serial.print(Output);
    Serial.println("");

    prevMicroSec = currentMicroSec;
  }
}


