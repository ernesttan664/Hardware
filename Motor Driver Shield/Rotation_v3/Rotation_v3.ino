/*
  Turning clockwise = increasing degrees and vice versa
  Robot will rotate clockwise (rotate right) on the spot
  
  Left motor = M2, Right motor = M1
  
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5
  
  Circumference of wheel = pi*d = 6*pi where d = diameter (cm)
  360 degree rotation = 3.25 revolution of the wheel (approx) = 19.5*pi cm
  Therefore full speed = 19.5*pi/1.25 = 49cm/s
  
  WARNING: Rotate() only works for degrees between 720 - 1080. Not accurate for degrees below 720
  NOTE: Accuracy is also dependent on amount of remaining battery power
  
*/


// Initialization
#define pi 3.14159265359
#define left_rotation_interval 1100
#define left_rotation_pi_const 0.9
#define right_rotation_interval 1100
#define right_rotation_pi_const 0.95
#define rotation_sampling_interval 100
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
double right_rotation_dc[right_rotation_interval/rotation_sampling_interval] ;
double left_rotation_dc[right_rotation_interval/rotation_sampling_interval] ;
void setup()
{
  
  Serial.begin(9600);
  Serial.println("Connection Established");
  for(int i=0,t=0; i<(right_rotation_interval/rotation_sampling_interval); i++,t+=rotation_sampling_interval){
    right_rotation_dc[i] = 0.5*sin(right_rotation_pi_const*pi*t*0.001)*sin(right_rotation_pi_const*pi*t*0.001);
    Serial.println(right_rotation_dc[i]);
  }
  Serial.println();
  for(int i=0,t=0; i<(left_rotation_interval/rotation_sampling_interval); i++,t+=rotation_sampling_interval){
    double dc = 0.5*sin(left_rotation_pi_const*pi*t*0.001)*sin(left_rotation_pi_const*pi*t*0.001);
    left_rotation_dc[i] =dc;
    Serial.println(dc);
  }
}

void loop()
{
  rotateAntiClockwise90();
  delay(2000);
  rotateClockwise90();
  delay(2000);
  
  //rotateLeft(720);
  //delay(2000);
  //rotateRight(720);
  //delay(2000);
}
void rotateAntiClockwise90(){
  // setting wheel direction to rotate robot clockwise
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  for(int i=0; i<(left_rotation_interval/rotation_sampling_interval); i++){
    analogWrite(m2PWM,left_rotation_dc[i]*255);
    analogWrite(m1PWM,left_rotation_dc[i]*255);
    delay(rotation_sampling_interval);
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);

}

void rotateClockwise90(){
  // setting wheel direction to rotate robot clockwise
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, LOW);
  
  for(int i=0; i<(right_rotation_interval/rotation_sampling_interval); i++){
    //Serial.println(dc);
    analogWrite(m2PWM,right_rotation_dc[i]*255);
    analogWrite(m1PWM,right_rotation_dc[i]*255);
    delay(rotation_sampling_interval);
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);

}
