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

int count=0;

void setup(){
  
  Serial.begin(115200);
  Serial.println("Connection Established");
  attachInterrupt(1, interrrupt_count, RISING);
}

void loop(){
  rotate();
  delay(1000);
}

void rotate(){
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  
  analogWrite(m2PWM,0.15*255);
  analogWrite(m1PWM,0.15*255);
}

void interrrupt_count(){
  count++;
  Serial.println(count);
  if(count>450){
    analogWrite(m2PWM,0.0*255);
    analogWrite(m1PWM,0.0*255);
  }
  
}
