/*
  Left motor = M2, Right motor = M1
  
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5
  
                    Home (at 0.2 PWM)
  rotate left 90:   1780
*/

// Initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int rotationTime=1780;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  rotateLeft90();
  delay(1500);
}

void rotateLeft90(){
  // setting wheel direction to rotate robot clockwise
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM, 0.2*255);
  analogWrite(m1PWM, 0.2*255);
  
  delay(rotationTime);
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}
