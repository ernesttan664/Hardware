/*
  Turning clockwise = increasing degrees and vice versa
  Robot will rotate clockwise (rotate right) on the spot
  
  Left motor = M2, Right motor = M1
  
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5
  
  Rotation (Degrees)      ms
  90                      200(100%)  1595(20% at home)
  180                     550
  360                     1250
  
  Circumference of wheel = pi*d = 6*pi where d = diameter (cm)
  360 degree rotation = 3.25 revolution of the wheel (approx) = 19.5*pi cm
  Therefore full speed = 19.5*pi/1.25 = 49cm/s
  
  WARNING: Rotate() only works for degrees between 720 - 1080. Not accurate for degrees below 720
  NOTE: Accuracy is also dependent on amount of remaining battery power
  
*/

// Initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int rightRotationTime=1595, leftRotationTime=1780;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  rotateRight90();
  delay(1500);
//  rotateLeft90();
//  delay(1500);
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
}
